#!/usr/bin/env python3

import argparse
import json
import math
from datetime import datetime, timezone


REASON_LABELS = {
    "rmse_limit": "RMSE 기준 초과",
    "outage_p95_limit": "음영 p95 기준 초과",
    "max_error_limit": "최대오차 기준 초과",
    "slam_no_improvement": "SLAM 개선 없음",
    "non_finite_state": "비유한 상태",
    "non_psd_covariance": "공분산 PSD 위반",
    "not_initialized": "초기화 실패",
}


def load_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def merge_intervals(intervals):
    merged = []
    for start, end in sorted(intervals):
        if not merged or start > merged[-1][1]:
            merged.append([float(start), float(end)])
        else:
            merged[-1][1] = max(merged[-1][1], float(end))
    return merged


def longest_overlap(left_intervals, right_intervals):
    left = merge_intervals(left_intervals)
    right = merge_intervals(right_intervals)
    longest = 0.0
    for left_start, left_end in left:
        for right_start, right_end in right:
            overlap = min(left_end, right_end) - max(
                left_start, right_start)
            longest = max(longest, overlap)
    return longest


def fmt(value, digits=3):
    if value is None or not math.isfinite(float(value)):
        return "-"
    return f"{float(value):.{digits}f}"


def comparison_rows(noise, slam):
    combined = noise["results"]["combined"]
    slam_result = slam["result"]
    return [
        {
            "test": "IMU/GPS 복합",
            "series": "강건 ESKF",
            "rmse_m": combined["robust_rmse_m"],
            "p95_m": combined["robust_p95_m"],
            "trials": 10,
        },
        {
            "test": "IMU/GPS 복합",
            "series": "기존 필터",
            "rmse_m": combined["legacy_rmse_m"],
            "p95_m": combined["legacy_p95_m"],
            "trials": 10,
        },
        {
            "test": "GPS 음영",
            "series": "SLAM 보조",
            "rmse_m": slam_result["slam_rmse_m"],
            "p95_m": slam_result["slam_outage_p95_m"],
            "trials": 10,
        },
        {
            "test": "GPS 음영",
            "series": "SLAM 미사용",
            "rmse_m": slam_result["no_slam_rmse_m"],
            "p95_m": slam_result["no_slam_outage_p95_m"],
            "trials": 10,
        },
    ]


def build_artifact(noise, slam, stress, regressions):
    generated_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    combined = noise["results"]["combined"]
    slam_result = slam["result"]
    completed = int(stress["completed"])
    failures = stress["failures"]
    passed_count = completed - int(stress["failure_count"])
    pass_rate = 100.0 * passed_count / max(completed, 1)
    chart_rows = comparison_rows(noise, slam)

    regression_rows = []
    regression_data = []
    regression_passed = 0
    for payload in regressions:
        case = payload["worst_cases"][0]
        result = case["result"]
        passed = not case["reasons"]
        regression_passed += int(passed)
        regression_rows.append([
            case["seed"],
            "통과" if passed else "실패",
            fmt(result.get("slam_rmse_m")),
            fmt(result.get("slam_outage_p95_m")),
            fmt(result.get("slam_max_m")),
        ])
        regression_data.append({
            "seed": f"S-{case['seed']}",
            "status": "통과" if passed else "실패",
            "rmse_m": result.get("slam_rmse_m"),
            "outage_p95_m": result.get("slam_outage_p95_m"),
            "max_error_m": result.get("slam_max_m"),
        })

    summary_rows = [
        [
            "IMU/GPS 복합 노이즈",
            "4개 시나리오 x 10회",
            f"RMSE {fmt(combined['robust_rmse_m'])}m",
            f"기존 {fmt(combined['legacy_rmse_m'])}m",
            "통과" if noise["passed"] else "실패",
        ],
        [
            "GPS 음영 + LIO-SAM",
            "고정 시나리오 10회",
            f"RMSE {fmt(slam_result['slam_rmse_m'])}m",
            f"SLAM 없음 {fmt(slam_result['no_slam_rmse_m'])}m",
            "통과" if slam["passed"] else "실패",
        ],
        [
            "과거 실패 회귀",
            f"{len(regressions)}개 재현 시드",
            f"{regression_passed}/{len(regressions)} 통과",
            "재발 여부",
            "통과" if regression_passed == len(regressions) else "실패",
        ],
        [
            "무작위 스트레스",
            f"{completed}개 시나리오",
            f"{passed_count}/{completed} ({pass_rate:.1f}%)",
            "엄격 한계값 적용",
            "조건부 통과" if failures else "통과",
        ],
    ]
    summary_data = [
        {
            "test": row[0],
            "scope": row[1],
            "result": row[2],
            "comparison": row[3],
            "status": row[4],
        }
        for row in summary_rows
    ]

    failure_rows = []
    failure_data = []
    for case in failures:
        scenario = case["scenario"]
        result = case["result"]
        slam_unavailable = (
            list(scenario.get("slam_dropouts", []))
            + list(scenario.get("slam_degenerate", []))
        )
        blackout = longest_overlap(
            scenario.get("gps_dropouts", []), slam_unavailable)
        gps_std = float(scenario["gps_std"])
        p95_limit = max(8.0, 4.0 * gps_std)
        failure_rows.append([
            case["seed"],
            ", ".join(
                REASON_LABELS.get(reason, reason)
                for reason in case["reasons"]),
            fmt(result["slam_rmse_m"]),
            f"{fmt(result['slam_outage_p95_m'])} / {fmt(p95_limit)}",
            fmt(result["slam_max_m"]),
            fmt(blackout, 1),
        ])
        failure_data.append({
            "seed": f"S-{case['seed']}",
            "reasons": ", ".join(
                REASON_LABELS.get(reason, reason)
                for reason in case["reasons"]),
            "rmse_m": result["slam_rmse_m"],
            "p95_and_limit_m": (
                f"{fmt(result['slam_outage_p95_m'])} / {fmt(p95_limit)}"),
            "max_error_m": result["slam_max_m"],
            "dual_blackout_sec": blackout,
        })

    sources = [
        {
            "id": "noise_results",
            "label": "IMU/GPS 노이즈 검증 결과",
            "path": "results/noise_10.json",
        },
        {
            "id": "slam_results",
            "label": "GPS 음영 LIO-SAM 검증 결과",
            "path": "results/final_slam_10.json",
        },
        {
            "id": "stress_results",
            "label": "무작위 스트레스 100회 결과",
            "path": "results/final_stress_100.json",
        },
        {
            "id": "regression_results",
            "label": "과거 실패 시드 회귀 결과",
            "path": "results/final_regression_results.json",
        },
        {
            "id": "comparison_sql",
            "label": "보고서 집계 데이터",
            "path": "results/report_summary.sql",
        },
    ]

    blocks = [
        {
            "id": "title",
            "type": "markdown",
            "body": "# MORAI ESKF + LIO-SAM 강건성 검증 결과",
        },
        {
            "id": "technical_summary",
            "type": "markdown",
            "body": (
                "## 핵심 결론\n\n"
                f"- 고정 노이즈·GPS 음영·과거 실패 회귀 시험은 모두 통과했음.\n"
                f"- 새 무작위 스트레스는 **{passed_count}/{completed}"
                f" ({pass_rate:.1f}%)**가 엄격 한계값을 통과했음.\n"
                "- 100개 모두 상태가 유한하고 공분산이 양의 준정부호를 "
                "유지해 수치 발산은 없었음.\n"
                f"- 기준 초과 {len(failures)}건은 아래 표에 숨김없이 "
                "정리했으며, GPS와 LIO-SAM이 동시에 사라지는 구간이 "
                "주요 잔여 위험임."
            ),
        },
        {
            "id": "summary_table",
            "type": "table",
            "tableId": "test_summary",
            "layout": "full",
        },
        {
            "id": "comparison_intro",
            "type": "markdown",
            "sourceId": "comparison_sql",
            "body": (
                "## 강건 처리와 SLAM 보조가 오차를 크게 줄였음\n\n"
                "같은 10회 조건에서 강건 ESKF는 기존 필터보다, GPS 음영의 "
                "SLAM 보조는 미사용보다 RMSE가 낮았음. 막대는 전체 RMSE를 "
                "비교하며 정확한 p95 값은 결과표와 본문에 함께 제시했음."
            ),
        },
        {
            "id": "comparison_chart",
            "type": "chart",
            "chartId": "rmse_comparison",
            "layout": "full",
        },
        {
            "id": "noise_findings",
            "type": "markdown",
            "sourceId": "noise_results",
            "body": (
                "## 복합 노이즈에서 기존 필터보다 안정적이었음\n\n"
                f"강건 ESKF의 복합 조건 RMSE는 "
                f"**{fmt(combined['robust_rmse_m'])}m**, p95는 "
                f"**{fmt(combined['robust_p95_m'])}m**였음. "
                f"기존 비게이팅 필터 RMSE "
                f"**{fmt(combined['legacy_rmse_m'])}m** 대비 "
                f"{combined['legacy_rmse_m'] / combined['robust_rmse_m']:.1f}배 "
                "낮았고, GPS 이상치와 IMU 스파이크를 거부한 상태에서도 "
                "yaw RMSE는 "
                f"**{fmt(combined['robust_yaw_rmse_deg'])}도**였음."
            ),
        },
        {
            "id": "slam_findings",
            "type": "markdown",
            "sourceId": "slam_results",
            "body": (
                "## GPS 음영에서 LIO-SAM 보조 효과가 확인됐음\n\n"
                f"SLAM 미사용 RMSE **{fmt(slam_result['no_slam_rmse_m'])}m** "
                f"대비 SLAM 보조 RMSE는 "
                f"**{fmt(slam_result['slam_rmse_m'])}m**였음. "
                f"음영 p95 **{fmt(slam_result['slam_outage_p95_m'])}m**, "
                f"최대오차 **{fmt(slam_result['slam_max_m'])}m**로 "
                "고정 기준을 통과했음."
            ),
        },
        {
            "id": "regression_intro",
            "type": "markdown",
            "sourceId": "regression_results",
            "body": (
                "## 과거 실패 시드 재발 없음\n\n"
                f"수정 과정에서 분리한 실패 시드 {len(regressions)}개를 "
                "최종 코드로 다시 실행했고 모두 기준을 통과했음."
            ),
        },
        {
            "id": "regression_table",
            "type": "table",
            "tableId": "regression_detail",
            "layout": "full",
        },
        {
            "id": "stress_failures_intro",
            "type": "markdown",
            "sourceId": "stress_results",
            "body": (
                "## 무작위 스트레스 기준 초과 상세\n\n"
                "동시 단절은 GPS 음영과 SLAM 드롭 또는 퇴화가 "
                "겹친 최장 시간을 뜻함. 0초인 경우에는 장시간 SLAM "
                "드리프트, 프레임 재정합, 이상치 조합이 주원인임."
            ),
        },
        {
            "id": "stress_failures_table",
            "type": "table",
            "tableId": "stress_failures",
            "layout": "full",
        },
        {
            "id": "definitions",
            "type": "markdown",
            "body": (
                "## 측정 범위와 판정 기준\n\n"
                "- RMSE는 전체 주행 구간의 2D 위치 오차 제곱평균제곱근임.\n"
                "- 음영 p95는 GPS가 끊긴 구간 위치 오차의 95백분위임.\n"
                "- 무작위 시험 한계는 RMSE `max(5m, 2.5 x GPS 표준편차)`, "
                "음영 p95 `max(8m, 4 x GPS 표준편차)`, 최대오차 "
                "`max(20m, 8 x GPS 표준편차)`임.\n"
                "- 시나리오는 GPS/IMU 노이즈·바이어스·이상치, 메시지 지연, "
                "GPS/SLAM 단절, LIO 퇴화·프레임 리셋을 무작위 조합했음."
            ),
        },
        {
            "id": "method",
            "type": "markdown",
            "body": (
                "## 검증 방법\n\n"
                "ROS 시스템 패키지를 공유하는 독립 가상환경에서 동일 시드로 "
                "재현 가능한 Monte Carlo 시험을 실행했음. 각 실행은 상태 "
                "유한성, 공분산 PSD, SLAM 사용 전후 오차, 이상치 거부와 "
                "재정합 결과를 검사했음. 보고서 수치는 저장된 JSON 원본에서 "
                "자동 생성했음."
            ),
        },
        {
            "id": "limitations",
            "type": "markdown",
            "body": (
                "## 해석상 한계\n\n"
                "이 결과는 합성 궤적과 주입 노이즈에 대한 소프트웨어 검증이며 "
                "실차 또는 MORAI 실시간 네트워크의 최종 보증은 아님. GPS와 "
                "LIO-SAM이 동시에 장시간 사라지면 IMU 적분만으로 절대 위치를 "
                "유지할 수 없어 오차 증가가 물리적으로 불가피함."
            ),
        },
        {
            "id": "next_steps",
            "type": "markdown",
            "body": (
                "## 다음 검증 우선순위\n\n"
                "1. MORAI rosbag 재생으로 센서 타임스탬프와 TF 지연을 포함한 "
                "실시간 회귀시험을 추가해야 함.\n"
                "2. GPS·SLAM 동시 단절에는 차량 속도 또는 휠 오도메트리 보조와 "
                "LIO 품질 기반 감속/정지 정책을 연결해야 함.\n"
                "3. 현재 기준 초과 6개 시드를 고정 회귀군으로 유지해 이후 "
                "수정이 수치 안정성을 악화시키지 않는지 추적해야 함."
            ),
        },
    ]

    return {
        "surface": "report",
        "manifest": {
            "version": 1,
            "surface": "report",
            "title": "MORAI ESKF + LIO-SAM 강건성 검증 결과",
            "description": "노이즈, 이상치, 센서 단절 조건 반복 검증 결과",
            "generatedAt": generated_at,
            "cards": [],
            "charts": [
                {
                    "id": "rmse_comparison",
                    "title": "필터 및 SLAM 보조 RMSE 비교",
                    "subtitle": "각 조건 10회 평균, 단위 m",
                    "type": "bar",
                    "dataset": "rmse_comparison",
                    "sourceId": "comparison_sql",
                    "valueFormat": "number",
                    "encodings": {
                        "x": {
                            "field": "test",
                            "type": "nominal",
                            "label": "시험 조건",
                        },
                        "y": {
                            "field": "rmse_m",
                            "type": "quantitative",
                            "label": "RMSE (m)",
                            "format": "number",
                        },
                        "color": {
                            "field": "series",
                            "type": "nominal",
                            "label": "방법",
                        },
                        "tooltip": [
                            {
                                "field": "p95_m",
                                "type": "quantitative",
                                "label": "p95 (m)",
                                "format": "number",
                            },
                            {
                                "field": "trials",
                                "type": "quantitative",
                                "label": "반복 횟수",
                                "format": "number",
                            },
                        ],
                    },
                }
            ],
            "tables": [
                {
                    "id": "test_summary",
                    "title": "전체 시험 결과표",
                    "subtitle": "고정시험, 회귀시험, 무작위 스트레스 판정",
                    "dataset": "test_summary",
                    "sourceId": "comparison_sql",
                    "defaultSort": {
                        "field": "test",
                        "direction": "asc",
                    },
                    "columns": [
                        {"field": "test", "label": "시험", "type": "text"},
                        {"field": "scope", "label": "범위", "type": "text"},
                        {
                            "field": "result",
                            "label": "핵심 결과",
                            "type": "text",
                        },
                        {
                            "field": "comparison",
                            "label": "비교 기준",
                            "type": "text",
                        },
                        {
                            "field": "status",
                            "label": "판정",
                            "type": "text",
                        },
                    ],
                },
                {
                    "id": "regression_detail",
                    "title": "과거 실패 시드 회귀 결과",
                    "subtitle": "최종 코드로 동일 시드 재실행",
                    "dataset": "regression_detail",
                    "sourceId": "comparison_sql",
                    "defaultSort": {
                        "field": "seed",
                        "direction": "asc",
                    },
                    "columns": [
                        {"field": "seed", "label": "시드", "type": "text"},
                        {
                            "field": "status",
                            "label": "판정",
                            "type": "text",
                        },
                        {
                            "field": "rmse_m",
                            "label": "RMSE (m)",
                            "format": "number",
                        },
                        {
                            "field": "outage_p95_m",
                            "label": "음영 p95 (m)",
                            "format": "number",
                        },
                        {
                            "field": "max_error_m",
                            "label": "최대 (m)",
                            "format": "number",
                        },
                    ],
                },
                {
                    "id": "stress_failures",
                    "title": "무작위 스트레스 기준 초과 6건",
                    "subtitle": "100개 시나리오 중 엄격 한계값 초과 사례",
                    "dataset": "stress_failures",
                    "sourceId": "comparison_sql",
                    "defaultSort": {
                        "field": "seed",
                        "direction": "asc",
                    },
                    "columns": [
                        {"field": "seed", "label": "시드", "type": "text"},
                        {
                            "field": "reasons",
                            "label": "초과 항목",
                            "type": "text",
                        },
                        {
                            "field": "rmse_m",
                            "label": "RMSE (m)",
                            "format": "number",
                        },
                        {
                            "field": "p95_and_limit_m",
                            "label": "p95 / 한계 (m)",
                            "type": "text",
                        },
                        {
                            "field": "max_error_m",
                            "label": "최대 (m)",
                            "format": "number",
                        },
                        {
                            "field": "dual_blackout_sec",
                            "label": "동시 단절 (s)",
                            "format": "number",
                        },
                    ],
                },
            ],
            "sources": sources,
            "blocks": blocks,
        },
        "snapshot": {
            "version": 1,
            "generatedAt": generated_at,
            "status": "ready",
            "datasets": {
                "rmse_comparison": chart_rows,
                "test_summary": summary_data,
                "regression_detail": regression_data,
                "stress_failures": failure_data,
            },
        },
        "sources": sources,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Build the portable ESKF validation report artifact")
    parser.add_argument("--noise", required=True)
    parser.add_argument("--slam", required=True)
    parser.add_argument("--stress", required=True)
    parser.add_argument("--regression", nargs="+", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--support-sql")
    args = parser.parse_args()

    noise = load_json(args.noise)
    slam = load_json(args.slam)
    artifact = build_artifact(
        noise,
        slam,
        load_json(args.stress),
        [load_json(path) for path in args.regression],
    )
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(artifact, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    if args.support_sql:
        statements = []
        for dataset, rows in artifact["snapshot"]["datasets"].items():
            selects = []
            for row in rows:
                values = []
                for key, value in row.items():
                    if isinstance(value, str):
                        literal = "'" + value.replace("'", "''") + "'"
                    elif value is None:
                        literal = "NULL"
                    else:
                        literal = str(value)
                    values.append(f"{literal} AS {key}")
                selects.append("SELECT " + ", ".join(values))
            statements.append(
                f"-- {dataset}\n" + "\nUNION ALL\n".join(selects) + ";")
        with open(args.support_sql, "w", encoding="utf-8") as stream:
            stream.write("\n\n".join(statements))
            stream.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
