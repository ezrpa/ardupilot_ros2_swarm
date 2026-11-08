#!/usr/bin/env python3
"""Playwright end-to-end check of the live TAK viewer (SAR mission).

Drives a headless Chromium against the viewer while a mission runs and
asserts what an operator should see, stage by stage, saving a screenshot at
each stage:

  1. picture   : >= N drone tracks and the SEARCH AREA arrive over CoT
  2. searching : drones are claiming grid cells (BFS/DFS fronts visible)
  3. found     : SURVIVOR FOUND - CASEVAC marker + survivor card + GeoChat
  4. on scene  : every unit reports ON SCENE

    python3 tests/e2e/check_tak_viewer.py                    # localhost:8088
    python3 tests/e2e/check_tak_viewer.py --url http://VM:8088 --drones 4

Exit code 0 = all stages passed. Needs: pip install playwright &&
python3 -m playwright install chromium.
"""
import argparse
import os
import sys
import time

from playwright.sync_api import sync_playwright


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8088")
    ap.add_argument("--drones", type=int, default=4)
    ap.add_argument("--min-cells", type=int, default=6)
    ap.add_argument("--timeout", type=float, default=900, help="seconds")
    ap.add_argument("--out", default="test-results/tak_viewer")
    ap.add_argument("--headed", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    deadline = time.time() + args.timeout
    results, errors = [], []

    with sync_playwright() as p:
        browser = p.chromium.launch(headless=not args.headed)
        page = browser.new_page(viewport={"width": 1600, "height": 900})
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.on("console", lambda m: m.type == "error"
                and "tile" not in m.text.lower() and errors.append(m.text))
        page.goto(args.url, wait_until="domcontentloaded")

        def stage(name, js_pred, detail_js):
            while time.time() < deadline:
                if page.evaluate(js_pred):
                    detail = page.evaluate(detail_js)
                    shot = os.path.join(args.out, f"{len(results) + 1}_{name}.png")
                    page.mouse.move(1590, 450)
                    page.wait_for_timeout(1200)       # let the map settle
                    page.screenshot(path=shot)
                    results.append((name, "PASS", detail, shot))
                    print(f"PASS {name}: {detail}  [{shot}]", flush=True)
                    return True
                page.wait_for_timeout(2000)
            shot = os.path.join(args.out, f"FAIL_{name}.png")
            page.screenshot(path=shot)
            results.append((name, "FAIL", page.evaluate(detail_js), shot))
            print(f"FAIL {name}: timed out  [{shot}]", flush=True)
            return False

        status = "document.getElementById('status-txt').textContent"
        ok = stage(
            "picture",
            f"document.querySelectorAll('#drones .drone').length >= {args.drones}"
            " && document.querySelector('.area-lbl') !== null",
            f"document.querySelectorAll('#drones .drone').length + ' drones, ' + {status}")
        ok = ok and stage(
            "searching",
            f"/(\\d+)\\//.test(document.getElementById('k-cov-l').textContent)"
            f" && +document.getElementById('k-cov-l').textContent.split('/')[0] >= {args.min_cells}",
            "document.getElementById('algo').textContent + ': ' + "
            "document.getElementById('k-cov-l').textContent")
        ok = ok and stage(
            "found",
            "document.getElementById('survivor').classList.contains('show')"
            " && document.querySelector('.sv-lbl') !== null"
            " && [...document.querySelectorAll('#feed-list li.chat')].length > 0",
            f"{status} + ' | ' + document.getElementById('sv-text').textContent"
            " + ' @ ' + document.getElementById('sv-coord').textContent")
        ok = ok and stage(
            "on_scene",
            f"/ALL {args.drones} UNITS ON SCENE|MISSION COMPLETE|RETURNING/"
            f".test({status})",
            status)
        browser.close()

    print("\nSUMMARY")
    for name, res, detail, shot in results:
        print(f"  {res:4s} {name:10s} {detail}")
    if errors:
        print(f"  page errors: {errors[:5]}")
    sys.exit(0 if ok and not errors else 1)


if __name__ == "__main__":
    main()
