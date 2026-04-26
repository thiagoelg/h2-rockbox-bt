#             __________               __   ___.
#   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
#   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
#   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
#   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
#                     \/            \/     \/    \/            \/
#

BLUEDROID_DIR := $(ROOTDIR)/lib/bluedroid
BLUEDROID_SRC := $(call preprocess, $(BLUEDROID_DIR)/SOURCES)
BLUEDROID_OBJ := $(call c2obj, $(BLUEDROID_SRC))

OTHER_SRC += $(BLUEDROID_SRC)

LIBBLUEDROID := $(BUILDDIR)/lib/libbluedroid.a
CORE_LIBS := $(LIBBLUEDROID) $(CORE_LIBS) $(TLSFLIB)

INCLUDES += -I$(BLUEDROID_DIR)/export -I$(BLUEDROID_DIR)/include \
			-I$(BLUEDROID_DIR)/osi/include -I$(BLUEDROID_DIR)/sbc/decoder/include \
			-I$(BLUEDROID_DIR)/sbc/encoder/include -I$(BLUEDROID_DIR)/sbc/plc/include

BLUEDROID_FLAGS := $(CFLAGS) -Wno-undef -Wno-unused-parameter -Wno-format

$(BUILDDIR)/lib/bluedroid/%.o: $(BLUEDROID_DIR)/%.c
	$(SILENT)mkdir -p $(dir $@)
	$(call PRINTS,CC $(subst $(ROOTDIR)/,,$<))$(CC) \
		$(BLUEDROID_FLAGS) -c $< -o $@

$(LIBBLUEDROID): $(BLUEDROID_OBJ)
	$(SILENT)$(shell rm -f $@)
	$(call PRINTS,AR $(@F))$(AR) rcs $@ $^ >/dev/null