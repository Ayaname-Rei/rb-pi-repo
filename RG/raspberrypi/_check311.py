# -*- coding: utf-8 -*-
"""本地以 3.11 规则校验目标文件：Pi 是 3.11，本地是 3.12。

`ast.parse(feature_version=(3,11))` **抓不到 PEP 701**（已自检证实：它只管 match /
walrus 那几个语法开关，不管 f-string 词法），所以改用 3.12 的 tokenize 把 f-string
拆成 FSTRING_START/MIDDLE/END，再按 3.11 的三条限制逐一核对：

  ① 表达式里不能出现与外层 f-string 同引号的嵌套 f-string   f"{f"{x}"}"
  ② 表达式里不能出现反斜杠                                  f"{'\n'.join(a)}"
  ③ 表达式里不能出现 # 注释                                 f"{1 # c\n}"

先自检（三条已知的坏样本必须被捕获、好样本必须放过），自检不过直接不信任结果。
"""
import io
import sys
import token as T
import tokenize

# tokenize 里 f-string 相关的 token 类型（3.12+）
FSTRING_START = getattr(T, "FSTRING_START", None)
FSTRING_MIDDLE = getattr(T, "FSTRING_MIDDLE", None)
FSTRING_END = getattr(T, "FSTRING_END", None)

SAMPLES = [
    # (说明, 源码, 3.11 下是否合法)
    ("同引号嵌套 f-string", 'x = f"{f"{1}"}"\n', False),
    ("外层双引号内层单引号（合法）", 'x = f"{f\'{1}\'}"\n', True),
    ("表达式里带反斜杠", 'x = f"{\'a\\nb\'.strip()}"\n', False),
    ("表达式里带 # 注释", 'x = f"{1 # c\n}"\n', False),
    ("普通 f-string（合法）", 'x = f"{1}"\ny = f"{ {1: 2} }"\n', True),
    ("多行表达式（合法）", 'x = f"{(1 +\n 2)}"\n', True),
    ("字面文本里的反斜杠（合法）", 'x = f"a\\nb"\n', True),
    ("嵌套字典字面量（合法）", 'x = f"{\'a\': 1}["a"]"\n'.replace('"a"', "'a'"), True),
    ("三重引号 f-string（合法）", 'x = f"""{\'a\'}"""\n', True),
    ("format spec 里的嵌套（合法）", 'x = f"{1:{2}}"\n', True),
]


def scan(text):
    """返回 [(行号, 说明)]，即该文件在 3.11 下会报错的 PEP 701 用法。"""
    problems = []
    stack = []          # 每个元素 = 外层 f-string 的引号字符
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(text).readline))
    except tokenize.TokenError as e:
        return [(0, f"tokenize 失败: {e}")]

    for tok in toks:
        ttype, s, (sl, _) = tok.type, tok.string, tok.start
        if ttype == FSTRING_START:
            quote = s[-1]
            if stack and stack[-1] == quote:
                problems.append((sl, f"① 同引号 {quote!r} 嵌套 f-string（PEP 701）"))
            stack.append(quote)
        elif ttype == FSTRING_END:
            if stack:
                stack.pop()
        elif stack:
            # 以下都在某个 f-string 的表达式区域里
            if ttype == T.STRING and "\\" in s:
                problems.append((sl, "② f-string 表达式里的字符串带反斜杠（PEP 701）"))
            elif ttype == T.COMMENT:
                problems.append((sl, "③ f-string 表达式里的 # 注释（PEP 701）"))
    return problems


def selftest():
    print("== 检查器自检 ==")
    bad = 0
    for why, src, legal in SAMPLES:
        probs = scan(src)
        ok = (not probs) if legal else bool(probs)
        if ok:
            got = "通过" if legal else f"捕获 {probs[0][1]}"
            print(f"  OK {why}: {got}")
        else:
            want = "应通过但被误报" if legal else "应被捕获但漏检"
            print(f"  ✗ {why}: {want}（{probs}）")
            bad += 1
    return bad


def main():
    if FSTRING_START is None:
        sys.exit("本机 Python 太老，tokenize 不支持 f-string 子 token（需 3.12+）")
    if selftest():
        sys.exit("\n检查器自检未通过，结果不可信 —— 停止")
    print("\n== 目标文件校验（3.11 f-string 规则）==")
    bad = 0
    for path in sys.argv[1:]:
        text = io.open(path, encoding="utf-8").read()
        probs = scan(text)
        if probs:
            bad += 1
            print(f"  ✗ {path}: {len(probs)} 处")
            for ln, why in probs:
                print(f"      line {ln}: {why}")
                print(f"        {text.splitlines()[ln - 1].strip() if ln else ''}")
        else:
            print(f"  OK {path}（{len(text.splitlines())} 行）无 PEP 701 用法")
    sys.exit(f"\n{bad} 个文件在 Pi 3.11 上会报 SyntaxError" if bad else "\n全部通过")


if __name__ == "__main__":
    main()
