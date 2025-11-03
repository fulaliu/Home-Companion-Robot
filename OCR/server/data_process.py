# -*- coding: utf-8 -*-
import re
import json
import sys
from typing import List, Dict, Any, Tuple, Optional

# -------------------------
# 基础工具
# -------------------------
def norm(s: str) -> str:
    """去空白、全角冒号归一化"""
    return re.sub(r"\s+", " ", s.replace("：", ":").strip())

def up(s: str) -> str:
    return norm(s).upper()

def normalize_unit(u: Optional[str]) -> Optional[str]:
    """单位标准化：支持 EA, PC, PCS, EA/REEL；忽略大小写与空格"""
    if not u:
        return None
    u0 = up(u).replace(" ", "")
    if u0 in {"EA", "PC", "PCS", "EA/REEL"}:
        return u0
    if u0.replace(" ", "") == "EA/REEL":
        return "EA/REEL"
    return None

# -------------------------
# 标签识别（B 段四标签）
# -------------------------
def detect_label(token: str) -> Optional[str]:
    t = up(token)
    if re.fullmatch(r"P/?N:?", t):
        return "P/N"
    if re.fullmatch(r"QTY:?", t):
        return "QTY"
    if re.fullmatch(r"MPN:?", t):
        return "MPN"
    if re.fullmatch(r"BRAND:?", t):
        return "BRAND"
    return None

def extract_next_non_label(tokens: List[str], i: int) -> Tuple[Optional[str], Optional[int]]:
    j = i + 1
    while j < len(tokens):
        if detect_label(tokens[j]) is None:
            val = norm(tokens[j])
            if val:
                return tokens[j], j
        else:
            return None, None
        j += 1
    return None, None

# -------------------------
# QTY(+UNIT) 解析器
# -------------------------
QTY_LABEL_RE = re.compile(r"^Q['’]?TY:?", re.I)

def parse_qty_value_and_unit_from_tokens(tokens: List[str], start_idx: int) -> Tuple[Optional[int], Optional[str], int]:
    """
    从 start_idx 起解析数量与单位。返回 (qty_int, unit_norm, last_index_consumed)
    支持融合/拆分、多 token 邻接的常见 OCR 情况。
    """
    n = len(tokens)
    i = start_idx
    window = []
    for k in range(i, min(i + 3, n)):
        window.append(tokens[k])

    # 把可能带标签/冒号的 token 清理为碎片 pieces
    pieces: List[str] = []
    for w in window:
        w_norm = norm(w)
        if QTY_LABEL_RE.match(up(w_norm)):
            w_norm = QTY_LABEL_RE.sub("", w_norm).strip()
        if w_norm:
            pieces += w_norm.split()

    def try_unit_from_text(text: str) -> Optional[str]:
        # 去掉数字后剩下的字母/斜杠作为单位候选
        u = re.sub(r"[0-9,.\-]+", "", up(text)).strip()
        if not u and re.search(r"[A-Za-z/]", text):
            u = "".join(ch for ch in up(text) if ch.isalpha() or ch == "/")
        return normalize_unit(u) if u else None

    qty_int: Optional[int] = None
    unit: Optional[str] = None
    last_idx_used = start_idx

    # 找第一个数字
    qty_token_idx = -1
    for idx, p in enumerate(pieces):
        digits = re.sub(r"[^0-9]", "", p)
        if digits:
            qty_int = int(digits)
            qty_token_idx = idx
            break
    if qty_int is not None:
        # 先尝试从同一片段里取单位
        unit = try_unit_from_text(pieces[qty_token_idx]) or None
        # 若没有，再看紧随片段
        if unit is None and qty_token_idx + 1 < len(pieces):
            unit = normalize_unit(pieces[qty_token_idx + 1])

    # 粗略估算消费到的原始 token 边界
    if len(window) >= 2 and ((qty_token_idx >= 1) or (unit is not None and qty_token_idx >= 0 and qty_token_idx + 1 >= 1)):
        last_idx_used = start_idx + 1
    if len(window) >= 3 and ((qty_token_idx >= 2) or (unit is not None and qty_token_idx + 1 >= 2)):
        last_idx_used = start_idx + 2

    return qty_int, unit, min(last_idx_used, n - 1)

# -------------------------
# 找 B 段：P/N → QTY(+UNIT) → MPN → BRAND
# -------------------------
def find_block_B(tokens: List[str]) -> Tuple[Optional[Tuple[int, int]], Dict[str, Any]]:
    n = len(tokens)
    try:
        start = next(i for i, t in enumerate(tokens) if detect_label(t) == "P/N")
    except StopIteration:
        return None, {}
    captured = {"P/N": None, "QTY": None, "UNIT": None, "MPN": None, "BRAND": None}
    order = ["P/N", "QTY", "MPN", "BRAND"]
    k = 0
    i = start
    last = start

    while i < n and k < len(order):
        lab = detect_label(tokens[i])
        if lab == order[k]:
            if lab == "QTY":
                qty_val, qty_unit, last_idx_used = parse_qty_value_and_unit_from_tokens(tokens, i)
                if qty_val is None:
                    qty_val, qty_unit, last_idx_used = parse_qty_value_and_unit_from_tokens(tokens, i + 1)
                if qty_val is not None:
                    captured["QTY"] = qty_val
                    captured["UNIT"] = qty_unit
                    last = max(last, last_idx_used)
                    k += 1
                    i = last_idx_used + 1
                    continue
            else:
                val_tok, val_idx = extract_next_non_label(tokens, i)
                if val_tok is not None:
                    captured[lab] = norm(val_tok)
                    last = max(last, val_idx)
                    k += 1
                    i = val_idx + 1
                    continue
        i += 1
    if k < len(order):
        return None, {}
    return (start, last), captured

# -------------------------
# A 段里解析 PN 与 QTY(+UNIT)
# -------------------------
def parse_pn_from_A(A: List[str]) -> Tuple[Optional[str], Optional[str]]:
    # 融合 MFG.P/N:xxx / P/N:xxx / PN:xxx；排除 "Customer P/N"
    for i, tok in enumerate(A):
        t = up(tok)
        m = re.match(r"^(MFG\.P/?N|PN|P/?N)[:]?\s*(.+)$", t)
        if m:
            key_raw = m.group(1)
            if up(tok).startswith("CUSTOMER P/N"):
                continue
            val = tok.split(":", 1)[-1] if ":" in tok else m.group(2)
            key = "MFG.P/N" if key_raw.startswith("MFG") else ("P/N" if key_raw.startswith("P") else "PN")
            return key, norm(val)
        if t in {"MFG.P/N", "PN", "P/N"} and not up(tok).startswith("CUSTOMER P/N"):
            j = i + 1
            while j < len(A):
                if detect_label(A[j]) is None:
                    v = norm(A[j])
                    if v:
                        key = "MFG.P/N" if t == "MFG.P/N" else ("P/N" if t == "P/N" else "PN")
                        return key, v
                j += 1
    return None, None

def parse_qty_from_A(A: List[str]) -> Tuple[Optional[str], Optional[int], Optional[str]]:
    n = len(A)
    for i, tok in enumerate(A):
        t = up(tok)
        if re.match(r"^Q['’]?TY:?", t):
            qty, unit, _ = parse_qty_value_and_unit_from_tokens(A, i)
            if qty is None:
                qty, unit, _ = parse_qty_value_and_unit_from_tokens(A, i + 1)
            if qty is not None:
                return "QTY", qty, unit
    return None, None, None

# -------------------------
# 改进版：数量包含性检查（带 debug 可开）
# -------------------------
def contains_qty(A: List[str], qty: int, debug: bool = False) -> bool:
    """
    判断 A 段中是否包含与 qty 数值等价的数量（忽略千分位分隔符与单位）。
    适配单 token 整行与相邻 token 拼接场景；避免从较长数字中截子串误判。
    """
    def log(msg: str):
        if debug:
            print(msg)

    qty_str = str(qty)
    number_group_re = re.compile(r"(?<!\d)(\d{1,3}(?:[., ]\d{3})+|\d+)(?!\d)")

    def normalize_group(g: str) -> Optional[int]:
        s = g.replace(" ", "")
        if "." in s:
            if not re.fullmatch(r"\d{1,3}(?:[.,]\d{3})+", s):
                # 小数，跳过
                return None
        digits = re.sub(r"\D+", "", s)
        if not digits:
            return None
        return int(digits)

    def matches_in_text(text: str) -> bool:
        for m in number_group_re.finditer(text):
            g = m.group(1)
            val = normalize_group(g)
            if val is None:
                continue
            if val == qty:
                return True
        return False

    # 逐 token
    for i, tok in enumerate(A):
        if matches_in_text(tok):
            return True
        # 相邻 token 组合兜底
        if i + 1 < len(A) and matches_in_text(tok + " " + A[i + 1]):
            return True

    # 全局兜底
    return matches_in_text(" ".join(A))

# -------------------------
# 包含性：MPN
# -------------------------
def contains_mpn(A: List[str], mpn: str) -> bool:
    a = "\n".join(up(x) for x in A)
    return up(mpn) in a

# -------------------------
# 主流程：严格 1–8 步（含 Step 5 容错）
# -------------------------
def compare_and_report(
    rec_texts: List[str],
    tolerance: bool = False,         # ← 容错开关
    qty_debug: bool = False          # ← contains_qty 的日志开关（默认不打印）
) -> Dict[str, Any]:

    result: Dict[str, Any] = {}
    result["rec_texts"] = rec_texts[:]

    # 2–3) B 段 & A 段
    block, captured = find_block_B(rec_texts)
    if not block:
        result.update({
            "B": [],
            "A": rec_texts[:],
            "error": "未找到以 P/N 开始并包含 QTY、MPN、Brand 的连续字段块B",
            "final": "无法判定（缺少必要字段）"
        })
        return result

    start, end = block
    B = rec_texts[start:end + 1]
    A = rec_texts[:start] + rec_texts[end + 1:]
    result["B"] = B
    result["A"] = A

    # 4) 提取 B 内 MPN/QTY(+UNIT)
    mpn_B = captured.get("MPN")
    qty_B = captured.get("QTY")
    unit_B = captured.get("UNIT")
    pn_B = captured.get("P/N")
    brand_B = captured.get("BRAND")

    result["extracted_from_B"] = {
        "P/N": pn_B,
        "QTY": qty_B,
        "UNIT": normalize_unit(unit_B),
        "MPN": mpn_B,
        "BRAND": brand_B
    }

    # 5–7) A 段解析
    pn_key_A, pn_val_A = parse_pn_from_A(A)
    qty_key_A, qty_val_A, unit_A = parse_qty_from_A(A)

    comparisons: Dict[str, Any] = {}

    # ---------------------------
    # Step 5（含容错机制）
    # ---------------------------
    if pn_val_A is not None and qty_val_A is not None and mpn_B is not None and qty_B is not None:
        pn_match = (up(pn_val_A) == up(mpn_B))
        qty_match = (qty_val_A == qty_B)

        unit_A_norm = normalize_unit(unit_A)
        unit_B_norm = normalize_unit(unit_B)
        unit_match = (unit_A_norm == unit_B_norm) if (unit_A_norm or unit_B_norm) else None  # 单位仅记录

        comparisons["PN_in_A"] = {"key": pn_key_A, "value": pn_val_A, "matches_MPN": pn_match}
        comparisons["QTY_in_A"] = {"key": qty_key_A, "value": qty_val_A, "matches_QTY": qty_match}
        comparisons["UNIT_compare_info"] = {"unit_in_A": unit_A_norm, "unit_in_B": unit_B_norm, "unit_match": unit_match}

        if pn_match and qty_match:
            step5_verdict = "结果匹配"
            result["step5_verdict"] = step5_verdict
            result["comparisons"] = comparisons
            result["final"] = step5_verdict
            return result

        # ---- 容错开关逻辑 ----
        if tolerance:
            fallback = {
                "enabled": True,
                "pn_containment_used": False,
                "qty_containment_used": False,
                "pn_containment_ok": None,
                "qty_containment_ok": None
            }

            # 对于不匹配的项，做包含性兜底
            ok = True
            if not pn_match:
                fallback["pn_containment_used"] = True
                pn_ok = contains_mpn(A, mpn_B)
                fallback["pn_containment_ok"] = pn_ok
                ok = ok and pn_ok

            if not qty_match:
                fallback["qty_containment_used"] = True
                qty_ok = contains_qty(A, qty_B, debug=qty_debug)
                fallback["qty_containment_ok"] = qty_ok
                ok = ok and qty_ok

            comparisons["tolerance"] = fallback

            if ok:
                step5_verdict = "结果匹配（容错命中）"
                result["step5_verdict"] = step5_verdict
                result["comparisons"] = comparisons
                result["final"] = step5_verdict
                return result
            else:
                step5_verdict = "结果不匹配，请检查！（容错仍未命中）"
                result["step5_verdict"] = step5_verdict
                result["comparisons"] = comparisons
                result["final"] = step5_verdict
                return result

        # 容错未开启，直接按原规则判定不匹配
        step5_verdict = "结果不匹配，请检查！"
        result["step5_verdict"] = step5_verdict
        result["comparisons"] = comparisons
        result["final"] = step5_verdict
        return result

    # ---------------------------
    # Step 6 / Step 7 / Step 8
    # ---------------------------
    step6_verdict = None
    if pn_val_A is None and mpn_B is not None:
        mpn_contain = contains_mpn(A, mpn_B)
        step6_verdict = "MPN匹配" if mpn_contain else "MPN不匹配，请检查！"
        comparisons["MPN_containment_in_A"] = {"needle": mpn_B, "contain": mpn_contain}
        result["step6_verdict"] = step6_verdict

    step7_verdict = None
    if qty_val_A is None and qty_B is not None:
        qty_contain = contains_qty(A, qty_B, debug=qty_debug)
        step7_verdict = "QTY匹配" if qty_contain else "QTY不匹配，请检查！"
        comparisons["QTY_containment_in_A"] = {"needle": qty_B, "contain": qty_contain}
        result["step7_verdict"] = step7_verdict

    # 单位随附返回（不影响判定）
    result["unit_in_A"] = normalize_unit(unit_A)
    result["unit_in_B"] = normalize_unit(unit_B)

    if step6_verdict or step7_verdict:
        ok6 = (step6_verdict in (None, "MPN匹配"))
        ok7 = (step7_verdict in (None, "QTY匹配"))
        result["final"] = "结果匹配" if (ok6 and ok7) else "结果不匹配"
    else:
        result["final"] = "无法判定（缺少必要字段）"

    result["comparisons"] = comparisons
    result["pn_in_A"] = {"key": pn_key_A, "value": pn_val_A}
    result["qty_in_A"] = {"key": qty_key_A, "value": qty_val_A}
    return result

# -------------------------
# 从 JSON 文件读取并执行
# -------------------------
def run_on_file(json_path: str, tolerance: bool = False, qty_debug: bool = False) -> Dict[str, Any]:
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    rec_texts = data.get("rec_texts", [])
    return compare_and_report(rec_texts, tolerance=tolerance, qty_debug=qty_debug)

# 命令行用法： python script.py your.json [--tolerance] [--qty-debug]
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法：python script.py <paddleocr输出json文件路径> [--tolerance] [--qty-debug]")
        sys.exit(1)
    json_path = sys.argv[1]
    tol = "--tolerance" in sys.argv[2:]
    dbg = "--qty-debug" in sys.argv[2:]
    res = run_on_file(json_path, tolerance=tol, qty_debug=dbg)
    print(json.dumps(res, ensure_ascii=False, indent=2))
