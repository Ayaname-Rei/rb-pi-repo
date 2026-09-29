[CmdletBinding()]
param(
    [string]$Port = 'COM3',
    [switch]$RestoreOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$factory = @{
    '2' = 500
    '3' = 177
    '4' = 129
    '5' = 408
    '6' = 500
}

function Get-FactoryReference {
    param([int]$Id)

    $key = [string]$Id
    if (-not $factory.ContainsKey($key)) {
        throw "Only ID2-ID6 are allowed."
    }
    return [int]$factory[$key]
}

function New-LeArmPacket {
    param(
        [byte]$Command,
        [byte[]]$Parameters = [byte[]]@()
    )

    return [byte[]]@(
        0x55, 0x55, (2 + $Parameters.Length), $Command
    ) + $Parameters
}

function Send-Position {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$Id,
        [int]$Position
    )

    $reference = Get-FactoryReference -Id $Id
    if ([Math]::Abs($Position - $reference) -gt 5) {
        throw "Position for ID$Id must stay within +/-5 of its factory reference."
    }

    [byte[]]$parameters = [byte[]]@(
        1,
        0xDC, 0x05,
        $Id,
        ($Position -band 0xFF), (($Position -shr 8) -band 0xFF)
    )
    [byte[]]$packet = New-LeArmPacket -Command 3 -Parameters $parameters
    $Serial.DiscardInBuffer()
    $Serial.Write($packet, 0, $packet.Length)
    Start-Sleep -Milliseconds 500
    Write-Host ("ID{0} target {1} sent" -f $Id, $Position)
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    9600,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.ReadTimeout = 100
$serial.WriteTimeout = 1000
$serial.DtrEnable = $false
$serial.RtsEnable = $false

try {
    $serial.Open()
    foreach ($id in 2..6) {
        $reference = Get-FactoryReference -Id $id
        if (-not $RestoreOnly) {
            Send-Position -Serial $serial -Id $id -Position ($reference + 5)
        }
        Send-Position -Serial $serial -Id $id -Position $reference
    }
    if ($RestoreOnly) {
        Write-Host 'ID2-ID6 factory references restored.'
    }
    else {
        Write-Host 'ID2-ID6 test sequence completed and restored to factory references.'
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
