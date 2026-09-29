[CmdletBinding()]
param(
    [string]$Port,
    [switch]$ListPorts,
    [switch]$ReadPositions,
    [switch]$RawPosition,
    [int]$MovePosition = -1
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Get-AvailablePorts {
    [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
}

function New-LeArmPacket {
    param(
        [byte]$Command,
        [byte[]]$Parameters = [byte[]]@()
    )

    [byte[]]$packet = [byte[]]@(
        0x55, 0x55, (2 + $Parameters.Length), $Command
    ) + $Parameters
    return $packet
}

function Read-LeArmFrame {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [int]$TimeoutMs = 1500,
        [int]$ExpectedCommand = -1
    )

    $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMs)
    $firstHeaderSeen = $false

    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $nextByte = $SerialPort.ReadByte()
        }
        catch [System.TimeoutException] {
            continue
        }

        if (-not $firstHeaderSeen) {
            $firstHeaderSeen = ($nextByte -eq 0x55)
            continue
        }

        if ($nextByte -ne 0x55) {
            $firstHeaderSeen = ($nextByte -eq 0x55)
            continue
        }

        try {
            $length = $SerialPort.ReadByte()
            if ($length -lt 2) {
                $firstHeaderSeen = $false
                continue
            }

            [byte[]]$body = New-Object byte[] ($length - 1)
            for ($index = 0; $index -lt $body.Length; $index++) {
                $body[$index] = [byte]$SerialPort.ReadByte()
            }

            $frame = [pscustomobject]@{
                Command = $body[0]
                Data = if ($body.Length -gt 1) { [byte[]]$body[1..($body.Length - 1)] } else { [byte[]]@() }
                Raw = [byte[]]@((0x55, 0x55, $length) + $body)
            }
            # A board can leave more than one response frame in the USB FIFO
            # (for example, retries of the version query).  Always restart
            # header detection after consuming one complete frame so the
            # second 0x55 cannot be mistaken for a length byte.
            $firstHeaderSeen = $false
            if ($ExpectedCommand -lt 0 -or $frame.Command -eq $ExpectedCommand) {
                return $frame
            }
        }
        catch [System.TimeoutException] {
            $firstHeaderSeen = $false
        }
    }

    throw "Timed out waiting for a LeArm response. Confirm the board is in PC mode (K1 once, two beeps), the selected COM port is correct, and no other program has the port open."
}

function Invoke-LeArmCommand {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [byte]$Command,
        [byte[]]$Parameters = [byte[]]@(),
        [switch]$ExpectResponse,
        [int]$ExpectedResponseCommand = -1,
        [int]$ResponseTimeoutMs = 1500
    )

    [byte[]]$packet = New-LeArmPacket -Command $Command -Parameters $Parameters
    $SerialPort.DiscardInBuffer()
    $SerialPort.Write($packet, 0, $packet.Length)

    if ($ExpectResponse) {
        return Read-LeArmFrame -SerialPort $SerialPort -ExpectedCommand $ExpectedResponseCommand -TimeoutMs $ResponseTimeoutMs
    }
}

function Get-LeArmVersion {
    param([System.IO.Ports.SerialPort]$SerialPort)

    $frame = $null
    for ($attempt = 1; $attempt -le 3; $attempt++) {
        try {
            $frame = Invoke-LeArmCommand -SerialPort $SerialPort -Command 1 -ExpectResponse -ExpectedResponseCommand 1
            break
        }
        catch {
            if ($attempt -eq 3) {
                throw
            }
            Start-Sleep -Milliseconds 100
        }
    }
    if ($frame.Command -ne 1 -or $frame.Data.Length -ne 2) {
        throw "Unexpected version response: $(([BitConverter]::ToString($frame.Raw)))"
    }

    return [pscustomobject]@{
        ServoType = $frame.Data[0]
        FirmwareVersion = $frame.Data[1]
    }
}

function Get-LeArmPositions {
    param([System.IO.Ports.SerialPort]$SerialPort)

    $frame = Invoke-LeArmCommand -SerialPort $SerialPort -Command 13 -ExpectResponse -ExpectedResponseCommand 13 -ResponseTimeoutMs 3000
    if ($frame.Command -ne 13 -or $frame.Data.Length -ne 18) {
        throw "Unexpected position response: $(([BitConverter]::ToString($frame.Raw)))"
    }

    $positions = [ordered]@{}
    for ($index = 0; $index -lt $frame.Data.Length; $index += 3) {
        $id = $frame.Data[$index]
        $rawPosition = ([int]$frame.Data[$index + 1]) -bor (([int]$frame.Data[$index + 2]) -shl 8)
        # PowerShell rejects an unsigned value above 32767 when casting to
        # Int16.  Decode the two's-complement representation explicitly.
        $position = if ($rawPosition -ge 0x8000) { $rawPosition - 0x10000 } else { $rawPosition }
        $positions["ID$id"] = $position
    }
    return [pscustomobject]$positions
}

function Set-LeArmPosition {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [ValidateRange(1, 6)]
        [byte]$Id,
        [ValidateRange(0, 700)]
        [int]$Position
    )

    [byte[]]$parameters = [byte[]]@(
        1,       # one servo
        0xDC, 0x05, # requested duration 1500 ms; factory bus firmware uses its safe direct path
        $Id,
        ($Position -band 0xFF), (($Position -shr 8) -band 0xFF)
    )
    Invoke-LeArmCommand -SerialPort $SerialPort -Command 3 -Parameters $parameters | Out-Null
}

if ($ListPorts) {
    $ports = @(Get-AvailablePorts)
    if ($ports.Count -eq 0) {
        Write-Host 'No serial ports are currently visible to Windows.'
    }
    else {
        Write-Host "Available serial ports: $($ports -join ', ')"
    }
    if (-not $Port) {
        exit 0
    }
}

if (-not $Port) {
    throw 'Specify the LeArm COM port, for example: .\id1_protocol_probe.ps1 -Port COM3'
}

$serial = [System.IO.Ports.SerialPort]::new($Port, 9600, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 100
$serial.WriteTimeout = 1000
$serial.DtrEnable = $false
$serial.RtsEnable = $false

try {
    try {
        $serial.Open()
    }
    catch [System.UnauthorizedAccessException] {
        throw "$Port is already open in another program. Close every LeArm V2.1, UartAssist, ROS serial node, and other serial-tool window before running this probe."
    }
    $version = Get-LeArmVersion -SerialPort $serial
    $typeName = switch ($version.ServoType) {
        1 { 'PWM' }
        2 { 'BUS' }
        default { "unknown ($($version.ServoType))" }
    }

    Write-Host "Firmware version: $($version.FirmwareVersion); servo type: $typeName"

    if ($version.ServoType -eq 1) {
        Write-Warning 'The board booted as PWM firmware. Power-cycle it, confirm the original ID1 BUS servo and its cable are present during boot, then press K1 once for PC mode. Do not run a BUS ID1 movement test in this state.'
    }

    if ($RawPosition) {
        [byte[]]$rawRequest = New-LeArmPacket -Command 13
        $serial.DiscardInBuffer()
        $serial.Write($rawRequest, 0, $rawRequest.Length)
        Start-Sleep -Milliseconds 2000
        $rawResponse = [System.Collections.Generic.List[byte]]::new()
        while ($serial.BytesToRead -gt 0) {
            try {
                $rawResponse.Add([byte]$serial.ReadByte())
            }
            catch [System.TimeoutException] {
                break
            }
        }
        if ($rawResponse.Count -eq 0) {
            Write-Host 'Raw position response: NO_BYTES'
        }
        else {
            Write-Host "Raw position response: $([BitConverter]::ToString($rawResponse.ToArray()))"
        }
    }

    if ($MovePosition -ge 0) {
        if ($version.ServoType -ne 2 -or $version.FirmwareVersion -ne 1) {
            throw 'Refusing movement unless the board reports factory BUS firmware (servo type 2, version 1).'
        }
        if ($MovePosition -lt 226 -or $MovePosition -gt 246) {
            throw 'The first physical probe only permits ID1 target positions 226..246. Do not move farther toward the factory open direction until the new claw endpoints are measured.'
        }

        Write-Warning "Sending one ID1 target position $MovePosition. Factory direct BUS commands may execute in about 20 ms despite the 1500 ms field. Stop power immediately if the claw hits a hard stop, chatters, or heats."
        Set-LeArmPosition -SerialPort $serial -Id 1 -Position $MovePosition
        Start-Sleep -Milliseconds 800
        Write-Host "ID1 target $MovePosition sent. Run -RawPosition next to compare feedback."
    }

    if (-not $RawPosition -and $ReadPositions -and ($version.ServoType -ne 2 -or $version.FirmwareVersion -ne 1)) {
        throw 'Refusing position or movement commands unless the board reports factory BUS firmware (servo type 2, version 1).'
    }

    if (-not $RawPosition -and $ReadPositions) {
        $positions = Get-LeArmPositions -SerialPort $serial
        Write-Host 'Reported positions:'
        $positions | Format-List | Out-Host
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
