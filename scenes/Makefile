.PHONY : all

#n.b. the '-y' sets autoexec scripts to 'on' so that driver expressions will work
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	BLENDER = /Applications/Blender.app/Contents/MacOS/blender -y
else ifeq ($(UNAME_S),Linux)
	BLENDER = ../../blender-2.90.0-linux64/blender -y
else
	BLENDER = blender
endif

EXPORT_MESHES=export-meshes.py
EXPORT_WALKMESHES=export-walkmeshes.py
EXPORT_SCENE=export-scene.py

DIST=../dist

all : \
	$(DIST)/house.pnct \
	$(DIST)/house.w \
	$(DIST)/house.scene \

$(DIST)/house.pnct : house.blend $(EXPORT_MESHES)
	$(BLENDER) --background --python $(EXPORT_MESHES) -- '$<':Platforms '$@'

$(DIST)/house.scene : house.blend $(EXPORT_SCENE)
	$(BLENDER) --background --python $(EXPORT_SCENE) -- '$<':Platforms '$@'

$(DIST)/house.w : house.blend $(EXPORT_WALKMESHES)
	$(BLENDER) --background --python $(EXPORT_WALKMESHES) -- '$<':WalkMeshes '$@'
