.PHONY: run headless usb trt

run:
	@echo "▶️  YOLO + JSONL log + HTTP stream at http://$$(hostname -I | awk '{print $$1}'):8080/video"
	MODEL=/home/barney/plevelai_ros/weights/best.pt \
	CAM=csi \
	SENSOR_ID=0 \
	LOG=$(HOME)/plevelai/detections.log \
	PORT=8080 \
	SERIAL_PORT=$(SERIAL_PORT) \
	BAUDRATE=$(BAUDRATE) \
	CONFIG=$(CONFIG) \
	DRY_RUN=$(DRY_RUN) \
	./scripts/runtime/launch_pipeline.sh

headless:
	@SHOW=0 ./scripts/runtime/quickstart.sh

usb:
	@CAM=usb USB_INDEX=0 SHOW=1 ./scripts/runtime/quickstart.sh

trt:
	@./scripts/build/export_trt.sh models/best.pt
