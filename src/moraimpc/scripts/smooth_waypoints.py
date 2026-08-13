#!/usr/bin/env python3
"""기록된 웨이포인트 후처리 스크립트.

기능:
  1. 정지 구간 제거 (중복점 제거)
  2. 이동평균 스무딩 (노이즈 제거)
  3. 등간격 리샘플링 (원하는 간격으로 재배치)
  4. 기어 전환점 정리 (D/R 경계 깔끔하게)

사용법:
  python3 smooth_waypoints.py [입력] [출력] [--spacing 0.2] [--window 5]

  예: python3 smooth_waypoints.py waypoints_recorded.json waypoints.json --spacing 0.2
"""
import argparse
import json
import math
import numpy as np
from scipy.interpolate import interp1d
from scipy.ndimage import uniform_filter1d


def load(path):
    with open(path) as f:
        return json.load(f)["waypoints"]


def remove_duplicates(wps, min_dist=0.02):
    """정지 구간 중복점 제거 (2cm 이내 동일점)"""
    out = [wps[0]]
    for w in wps[1:]:
        d = math.hypot(w["x"] - out[-1]["x"], w["y"] - out[-1]["y"])
        if d >= min_dist:
            out.append(w)
    return out


def split_by_gear(wps):
    """기어별 세그먼트 분리"""
    segments = []
    cur_gear = wps[0].get("gear", "D")
    seg = [wps[0]]
    for w in wps[1:]:
        g = w.get("gear", "D")
        if g != cur_gear:
            segments.append((cur_gear, seg))
            cur_gear = g
            seg = [w]
        else:
            seg.append(w)
    segments.append((cur_gear, seg))
    return segments


def smooth_segment(seg, window=5):
    """이동평균 스무딩"""
    if len(seg) < window:
        return seg
    x = np.array([w["x"] for w in seg])
    y = np.array([w["y"] for w in seg])
    x_s = uniform_filter1d(x, size=window, mode='nearest')
    y_s = uniform_filter1d(y, size=window, mode='nearest')
    # 시작/끝점은 원본 유지
    x_s[0], x_s[-1] = x[0], x[-1]
    y_s[0], y_s[-1] = y[0], y[-1]
    return [{"x": float(x_s[i]), "y": float(y_s[i])} for i in range(len(seg))]


def resample_segment(seg, spacing=0.2):
    """등간격 리샘플링"""
    if len(seg) < 2:
        return seg
    x = np.array([w["x"] for w in seg])
    y = np.array([w["y"] for w in seg])

    # 누적 거리 계산
    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.sqrt(dx**2 + dy**2)
    s = np.concatenate([[0], np.cumsum(ds)])
    total = s[-1]

    if total < spacing:
        return seg

    # 등간격 보간
    n_pts = max(2, int(round(total / spacing)) + 1)
    s_new = np.linspace(0, total, n_pts)

    fx = interp1d(s, x, kind='linear')
    fy = interp1d(s, y, kind='linear')

    x_new = fx(s_new)
    y_new = fy(s_new)

    return [{"x": round(float(x_new[i]), 6), "y": round(float(y_new[i]), 6)} for i in range(n_pts)]


def add_headings(wps):
    """heading 계산"""
    n = len(wps)
    if n < 2:
        return wps
    for i in range(n):
        if i == 0:
            h = math.atan2(wps[1]["y"] - wps[0]["y"], wps[1]["x"] - wps[0]["x"])
        elif i == n - 1:
            h = math.atan2(wps[-1]["y"] - wps[-2]["y"], wps[-1]["x"] - wps[-2]["x"])
        else:
            h = math.atan2(wps[i+1]["y"] - wps[i-1]["y"], wps[i+1]["x"] - wps[i-1]["x"])
        wps[i]["heading"] = round(h, 8)
    return wps


def main():
    parser = argparse.ArgumentParser(description="웨이포인트 후처리")
    parser.add_argument("input", help="입력 JSON 파일")
    parser.add_argument("output", nargs="?", default=None, help="출력 JSON 파일 (기본: 입력 덮어쓰기)")
    parser.add_argument("--spacing", type=float, default=0.2, help="리샘플링 간격 [m] (기본: 0.2)")
    parser.add_argument("--window", type=int, default=5, help="스무딩 윈도우 크기 (기본: 5)")
    parser.add_argument("--no-smooth", action="store_true", help="스무딩 비활성화")
    parser.add_argument("--max-kappa", type=float, default=0.0,
                        help="곡률 한계 [1/m] (>0이면 한계 초과 점은 추가 스무딩으로 평탄화). 권장 0.25 (vehicle κ_max=0.28)")
    parser.add_argument("--max-kappa-passes", type=int, default=10,
                        help="max-kappa 평탄화 최대 반복 횟수")
    args = parser.parse_args()

    if args.output is None:
        args.output = args.input

    wps = load(args.input)
    print(f"원본: {len(wps)}점")

    # 1. 중복점 제거
    wps = remove_duplicates(wps)
    print(f"중복 제거 후: {len(wps)}점")

    # 2. 기어별 세그먼트 분리
    segments = split_by_gear(wps)
    print(f"세그먼트: {len(segments)}개", [(g, len(s)) for g, s in segments])

    # 3. 세그먼트별 스무딩 + 리샘플링 (resample 결과 길이로 boundary 기록)
    result = []
    seg_boundaries = []
    for gear, seg in segments:
        if not args.no_smooth:
            seg = smooth_segment(seg, args.window)
        seg = resample_segment(seg, args.spacing)
        seg = add_headings(seg)
        for w in seg:
            w["gear"] = gear
        # D만 있으면 gear 생략 가능하지만 혼합 경로이므로 유지
        result.extend(seg)
        seg_boundaries.append(len(result))  # 누적 — 각 segment 끝 직후 인덱스
        print(f"  {gear}: {len(seg)}점 (간격 {args.spacing}m)")

    # 3.5. Segment 경계 stitching: D 끝과 R 시작 좌표 통일 (gap 제거)
    # 가장 자연스럽게: D 끝점을 R 시작점과 같은 위치로 (또는 반대) — 여기선 평균
    # 단 stitch 후 heading 재계산 (양 segment 끝부분 1점 영향)
    for k in range(len(seg_boundaries) - 1):
        i_end   = seg_boundaries[k] - 1   # 이전 segment 마지막 점
        i_start = seg_boundaries[k]       # 다음 segment 첫 점
        mx = (result[i_end]["x"] + result[i_start]["x"]) / 2
        my = (result[i_end]["y"] + result[i_start]["y"]) / 2
        result[i_end]["x"]   = round(mx, 6); result[i_end]["y"]   = round(my, 6)
        result[i_start]["x"] = round(mx, 6); result[i_start]["y"] = round(my, 6)
        # 인접 점 영향 받은 heading 재계산
        for off in [-1, 0]:
            i = i_end + off
            if 0 < i < len(result) - 1 and result[i].get("gear") == result[i+off].get("gear"):
                result[i]["heading"] = round(math.atan2(
                    result[i+1]["y"] - result[i-1]["y"],
                    result[i+1]["x"] - result[i-1]["x"]), 8)
        for off in [0, 1]:
            i = i_start + off
            if 0 < i < len(result) - 1 and result[i].get("gear") == result[i+off].get("gear"):
                result[i]["heading"] = round(math.atan2(
                    result[i+1]["y"] - result[i-1]["y"],
                    result[i+1]["x"] - result[i-1]["x"]), 8)
    print(f"segment 경계 stitching: {len(seg_boundaries)-1}개 boundary 통합")

    # 3.7. 곡률 한계 클램프 (반복 1-3-1 스무딩)
    if args.max_kappa > 0:
        for pass_i in range(args.max_kappa_passes):
            n = len(result)
            xs = np.array([w["x"] for w in result])
            ys = np.array([w["y"] for w in result])
            # 곡률 계산: κ = dθ/ds
            kappa = np.zeros(n)
            for i in range(1, n - 1):
                if result[i-1].get("gear") != result[i].get("gear") or \
                   result[i].get("gear") != result[i+1].get("gear"):
                    continue  # 세그먼트 경계 건너뜀
                dx1 = xs[i] - xs[i-1]; dy1 = ys[i] - ys[i-1]
                dx2 = xs[i+1] - xs[i]; dy2 = ys[i+1] - ys[i]
                ds = (math.hypot(dx1, dy1) + math.hypot(dx2, dy2)) * 0.5
                if ds < 1e-6: continue
                dth = math.atan2(dy2, dx2) - math.atan2(dy1, dx1)
                while dth >  math.pi: dth -= 2*math.pi
                while dth < -math.pi: dth += 2*math.pi
                kappa[i] = dth / ds
            n_over = int(np.sum(np.abs(kappa) > args.max_kappa))
            if n_over == 0:
                print(f"  pass {pass_i+1}: κ 위반 0개 — 종료")
                break
            print(f"  pass {pass_i+1}: κ>{args.max_kappa} 위반 {n_over}개 — 강한 5점 가우시안")
            xs_new = xs.copy(); ys_new = ys.copy()
            # 5점 [1,4,6,4,1]/16 가우시안 — 1-3-1보다 ~2배 강함
            for i in range(2, n - 2):
                if abs(kappa[i]) <= args.max_kappa: continue
                # 세그먼트 경계 ±2 ticks 내면 약한 3점만
                gear_i = result[i].get("gear")
                if any(result[j].get("gear") != gear_i for j in (i-2, i-1, i+1, i+2)):
                    if result[i-1].get("gear") == gear_i and result[i+1].get("gear") == gear_i:
                        xs_new[i] = (xs[i-1] + 2*xs[i] + xs[i+1]) / 4.0
                        ys_new[i] = (ys[i-1] + 2*ys[i] + ys[i+1]) / 4.0
                    continue
                xs_new[i] = (xs[i-2] + 4*xs[i-1] + 6*xs[i] + 4*xs[i+1] + xs[i+2]) / 16.0
                ys_new[i] = (ys[i-2] + 4*ys[i-1] + 6*ys[i] + 4*ys[i+1] + ys[i+2]) / 16.0
            for i in range(n):
                result[i]["x"] = round(float(xs_new[i]), 6)
                result[i]["y"] = round(float(ys_new[i]), 6)
        # 최종 heading 재계산
        for k_seg in range(len(seg_boundaries)):
            i_start = 0 if k_seg == 0 else seg_boundaries[k_seg-1]
            i_end = seg_boundaries[k_seg]
            for i in range(i_start, i_end):
                if i == i_start:
                    if i_end - i_start >= 2:
                        h = math.atan2(result[i+1]["y"] - result[i]["y"],
                                       result[i+1]["x"] - result[i]["x"])
                    else: h = result[i].get("heading", 0)
                elif i == i_end - 1:
                    h = math.atan2(result[i]["y"] - result[i-1]["y"],
                                   result[i]["x"] - result[i-1]["x"])
                else:
                    h = math.atan2(result[i+1]["y"] - result[i-1]["y"],
                                   result[i+1]["x"] - result[i-1]["x"])
                result[i]["heading"] = round(h, 8)

    # 4. 저장
    data = {"waypoints": result}
    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"저장: {args.output} ({len(result)}점)")


if __name__ == "__main__":
    main()
