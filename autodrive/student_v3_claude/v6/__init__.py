"""v6 판단 모델 그림자 실행 (학습 코드 student/ 에서 make_v6_package.py 로 생성 — 직접 고치지 말 것)."""
from .shadow_runtime import start_v6_shadow, stop_v6_shadow, notify_v6_idle, v6_record_fields  # noqa: F401
