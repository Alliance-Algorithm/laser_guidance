.PHONY: scan-camera set-config preview stream record stop competition

scan-camera:
	@.script/scan-camera

set-config:
	@.script/set-config

preview:
	@.script/preview

stream:
	@.script/stream

record:
	@.script/record

stop:
	@.script/stop

competition:
	@.script/competition
