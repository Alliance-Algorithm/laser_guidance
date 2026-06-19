.PHONY: build clean scan-camera set-config preview stream record stop competition

build:
	@.script/build-laser

clean:
	@.script/clean-laser

scan-camera:
	@.script/scan-camera

set-config:
	@.script/set-config

preview:
	@.script/preview-laser

stream:
	@.script/stream

record:
	@.script/record $(ARGS)

stop:
	@.script/stop

competition:
	@.script/competition-laser
