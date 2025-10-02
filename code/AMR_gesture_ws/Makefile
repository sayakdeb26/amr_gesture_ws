.PHONY: ros-build vlm-run lint fmt

ros-build:
	colcon build --merge-install --symlink-install

vlm-run:
	uvicorn cloud.video_llava_service.app:app --host 127.0.0.1 --port 8000

lint:
	pre-commit run --all-files

fmt:
	black .
