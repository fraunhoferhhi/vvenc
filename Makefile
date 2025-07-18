# VVenC/Makefile
#
# How to build a single target:
#  make <project>-r  => build variant=release
#  make <project>-d  => build variant=debug
#  make <project>-p  => build variant=relwithdebinfo
#
# How to clean and build a single target:
#  make <project>-cr  => clean + build variant=release
#  make <project>-cd  => clean + build variant=debug
#  make <project>-cp  => clean + build variant=relwithdebinfo
#

TARGETS := vvenc vvencapp vvencFFapp

ifneq ($(g),)
ifeq ($(g),umake)
CMAKE_GENERATOR_CUSTOM := Unix Makefiles
else ifeq ($(g),ninja)
CMAKE_GENERATOR_CUSTOM := Ninja
else ifeq ($(g),xcode)
CMAKE_GENERATOR_CUSTOM := Xcode
else
CMAKE_GENERATOR_CUSTOM := $(g)
endif
CONFIG_OPTIONS += -G '$(CMAKE_GENERATOR_CUSTOM)'
endif

ifneq ($(verbose),)
CONFIG_OPTIONS += -DCMAKE_VERBOSE_MAKEFILE=ON
endif

ifneq ($(enable-tracing),)
CONFIG_OPTIONS += -DVVENC_ENABLE_TRACING=$(enable-tracing)
endif

ifneq ($(address-sanitizer),)
CONFIG_OPTIONS += -DVVENC_USE_ADDRESS_SANITIZER=$(address-sanitizer)
endif

ifneq ($(thread-sanitizer),)
CONFIG_OPTIONS += -DVVENC_USE_THREAD_SANITIZER=$(thread-sanitizer)
endif

ifneq ($(enable-arch),)
CONFIG_OPTIONS += -DVVENC_OPT_TARGET_ARCH=$(enable-arch)
endif

ifneq ($(disable-lto),)
CONFIG_OPTIONS += -DVVENC_ENABLE_LINK_TIME_OPT=OFF
endif

ifneq ($(enable-json),)
CONFIG_OPTIONS += -DVVENC_ENABLE_THIRDPARTY_JSON=${enable-json}
endif

ifneq ($(enable-build-type-postfix),)
CONFIG_OPTIONS += -DVVENC_ENABLE_BUILD_TYPE_POSTFIX=ON
endif

ifneq ($(install-prefix),)
CONFIG_OPTIONS += -DCMAKE_INSTALL_PREFIX=$(install-prefix)
endif

ifneq ($(osx-arch),)
CONFIG_OPTIONS += -DCMAKE_OSX_ARCHITECTURES=$(osx-arch)
endif

ifneq ($(toolchainfile),)
CONFIG_OPTIONS += -DCMAKE_TOOLCHAIN_FILE=$(toolchainfile)
endif

ifneq ($(install-ffapp),)
CONFIG_OPTIONS += -DVVENC_INSTALL_FULLFEATURE_APP=$(install-ffapp)
endif

ifneq ($(override-compiler-check),)
CONFIG_OPTIONS += -DVVENC_OVERRIDE_COMPILER_CHECK=$(override-compiler-check)
endif

ifneq ($(enable-werror),)
CONFIG_OPTIONS += -DVVENC_ENABLE_WERROR=$(enable-werror)
endif

ifneq ($(enable-unstable-api),)
CONFIG_OPTIONS += -DVVENC_ENABLE_UNSTABLE_API=$(enable-unstable-api)
endif

ifneq ($(ffp-contract-off),)
CONFIG_OPTIONS += -DVVENC_FFP_CONTRACT_OFF=$(ffp-contract-off)
endif

ifeq ($(j),)
# Query cmake for the number of cores
NUM_JOBS := $(shell cmake -P cmake/modules/vvencNumCores.cmake)
NUM_JOBS := $(lastword $(NUM_JOBS))
else
NUM_JOBS := $(j)
endif

ifeq ($(OS),Windows_NT)
  ifneq ($(msvc-arch),)
    CONFIG_OPTIONS += -A $(msvc-arch)
  else
    CONFIG_OPTIONS += -A x64
  endif
  CMAKE_MCONFIG := 1
  ifneq ($(NUM_JOBS),1)
    BUILD_TOOL_OPTIONS := -- /maxcpucount:$(NUM_JOBS) /verbosity:minimal /nr:false
  else
    BUILD_TOOL_OPTIONS := -- /verbosity:minimal /nr:false
  endif
else
  UNAME_S := $(shell uname -s)
  ifeq ($(UNAME_S),Darwin)
    ifneq ($(CMAKE_GENERATOR_CUSTOM),)
      ifeq ($(CMAKE_GENERATOR_CUSTOM),Xcode)
        CMAKE_MCONFIG := 1
      endif
    else
      # No CMake generator specified as argument. Check for environment first.
      ifeq ($(CMAKE_GENERATOR),)
        CMAKE_MCONFIG := 1
        # Default depends wether ninja is installed or not but this cannot be determined easily.
        CONFIG_OPTIONS += -G Xcode
      else ifeq ($(CMAKE_GENERATOR),Xcode)
        CMAKE_MCONFIG := 1
      endif
    endif
  endif
  ifneq ($(NUM_JOBS),1)
    BUILD_JOBS := -j $(NUM_JOBS)
  endif
endif


ifeq ($(CMAKE_MCONFIG),)
# Using a CMake single-config generater like Unix Makefiles or Ninja
BUILD_DIR-release := build/release-static
BUILD_DIR-debug := build/debug-static
BUILD_DIR-relwithdebinfo := build/relwithdebinfo-static
BUILD_DIR-release-shared := build/release-shared
BUILD_DIR-debug-shared := build/debug-shared
BUILD_DIR-relwithdebinfo-shared := build/relwithdebinfo-shared
else
# Using a CMake multi-config generator like Visual Studio or Xcode
CONFIG_OPTIONS += -DCMAKE_CONFIGURATION_TYPES=Debug\;Release\;RelWithDebInfo

BUILD_DIR_STATIC := build/static
BUILD_DIR_SHARED := build/shared

BUILD_DIR-release := $(BUILD_DIR_STATIC)
BUILD_DIR-debug := $(BUILD_DIR_STATIC)
BUILD_DIR-relwithdebinfo := $(BUILD_DIR_STATIC)
BUILD_DIR-release-shared := $(BUILD_DIR_SHARED)
BUILD_DIR-debug-shared := $(BUILD_DIR_SHARED)
BUILD_DIR-relwithdebinfo-shared := $(BUILD_DIR_SHARED)
endif

BUILD_OPTIONS-release := --build $(BUILD_DIR-release)
BUILD_OPTIONS-debug := --build $(BUILD_DIR-debug)
BUILD_OPTIONS-relwithdebinfo := --build $(BUILD_DIR-relwithdebinfo)
BUILD_OPTIONS-release-shared := --build $(BUILD_DIR-release-shared)
BUILD_OPTIONS-debug-shared := --build $(BUILD_DIR-debug-shared)
BUILD_OPTIONS-relwithdebinfo-shared := --build $(BUILD_DIR-relwithdebinfo-shared)


ifneq ($(CMAKE_MCONFIG),)
BUILD_OPTIONS-release += --config Release
BUILD_OPTIONS-debug += --config Debug
BUILD_OPTIONS-relwithdebinfo += --config RelWithDebInfo
BUILD_OPTIONS-release-shared += --config Release
BUILD_OPTIONS-debug-shared += --config Debug
BUILD_OPTIONS-relwithdebinfo-shared += --config RelWithDebInfo
endif

DEFAULT_BUILD_TARGETS_STATIC := release debug relwithdebinfo
DEFAULT_BUILD_TARGETS_SHARED := $(foreach t,$(DEFAULT_BUILD_TARGETS_STATIC),$(t)-shared)
DEFAULT_BUILD_TARGETS := $(DEFAULT_BUILD_TARGETS_STATIC) $(DEFAULT_BUILD_TARGETS_SHARED)


release: $(BUILD_DIR-release)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-$@) $(BUILD_JOBS) $(BUILD_TOOL_OPTIONS)

debug: $(BUILD_DIR-debug)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-$@) $(BUILD_JOBS) $(BUILD_TOOL_OPTIONS)

relwithdebinfo: $(BUILD_DIR-relwithdebinfo)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-$@) $(BUILD_JOBS) $(BUILD_TOOL_OPTIONS)

release-shared: $(BUILD_DIR-release-shared)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-$@) $(BUILD_JOBS) $(BUILD_TOOL_OPTIONS)

debug-shared: $(BUILD_DIR-debug-shared)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-$@) $(BUILD_JOBS) $(BUILD_TOOL_OPTIONS)

relwithdebinfo-shared: $(BUILD_DIR-relwithdebinfo-shared)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-$@) $(BUILD_JOBS) $(BUILD_TOOL_OPTIONS)

$(foreach t,$(DEFAULT_BUILD_TARGETS),clean-$(t)):
	cmake $(BUILD_OPTIONS-$(patsubst clean-%,%,$@)) $(BUILD_JOBS) --target clean $(BUILD_TOOL_OPTIONS)

install-release: release
	cmake $(BUILD_OPTIONS-$(patsubst install-%,%,$@)) --target install

install-debug: debug
	cmake $(BUILD_OPTIONS-$(patsubst install-%,%,$@)) --target install

install-relwithdebinfo: relwithdebinfo
	cmake $(BUILD_OPTIONS-$(patsubst install-%,%,$@)) --target install

install-release-shared: release-shared
	cmake $(BUILD_OPTIONS-$(patsubst install-%,%,$@)) --target install

install-debug-shared: debug-shared
	cmake $(BUILD_OPTIONS-$(patsubst install-%,%,$@)) --target install

install-relwithdebinfo-shared: relwithdebinfo-shared
	cmake $(BUILD_OPTIONS-$(patsubst install-%,%,$@)) --target install


ifeq ($(CMAKE_MCONFIG),)
$(BUILD_DIR-release)/CMakeCache.txt configure-release:
	cmake -S . -B $(BUILD_DIR-release) $(CONFIG_OPTIONS) -DCMAKE_BUILD_TYPE=Release

$(BUILD_DIR-debug)/CMakeCache.txt configure-debug:
	cmake -S . -B $(BUILD_DIR-debug) $(CONFIG_OPTIONS) -DCMAKE_BUILD_TYPE=Debug

$(BUILD_DIR-relwithdebinfo)/CMakeCache.txt configure-relwithdebinfo:
	cmake -S . -B $(BUILD_DIR-relwithdebinfo) $(CONFIG_OPTIONS) -DCMAKE_BUILD_TYPE=RelWithDebInfo

$(BUILD_DIR-release-shared)/CMakeCache.txt configure-release-shared:
	cmake -S . -B $(BUILD_DIR-release-shared) $(CONFIG_OPTIONS) -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=1

$(BUILD_DIR-debug-shared)/CMakeCache.txt configure-debug-shared:
	cmake -S . -B $(BUILD_DIR-debug-shared) $(CONFIG_OPTIONS) -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=1

$(BUILD_DIR-relwithdebinfo-shared)/CMakeCache.txt configure-relwithdebinfo-shared:
	cmake -S . -B $(BUILD_DIR-relwithdebinfo-shared) $(CONFIG_OPTIONS) -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_SHARED_LIBS=1

configure-static: $(foreach t,$(DEFAULT_BUILD_TARGETS_STATIC),configure-$(t))
configure-shared: $(foreach t,$(DEFAULT_BUILD_TARGETS_SHARED),configure-$(t))

else
$(BUILD_DIR_STATIC)/CMakeCache.txt configure-static $(foreach t,$(DEFAULT_BUILD_TARGETS_STATIC),configure-$(t)):
	cmake -S . -B $(BUILD_DIR_STATIC) $(CONFIG_OPTIONS)

$(BUILD_DIR_SHARED)/CMakeCache.txt configure-shared $(foreach t,$(DEFAULT_BUILD_TARGETS_SHARED),configure-$(t)):
	cmake -S . -B $(BUILD_DIR_SHARED) $(CONFIG_OPTIONS) -DBUILD_SHARED_LIBS=1
endif

static: $(DEFAULT_BUILD_TARGETS_STATIC)
shared: $(DEFAULT_BUILD_TARGETS_SHARED)

all: static shared

configure: configure-static configure-shared

install-static: $(foreach t,$(DEFAULT_BUILD_TARGETS_STATIC),install-$(t))
install-shared: $(foreach t,$(DEFAULT_BUILD_TARGETS_SHARED),install-$(t))
install-all: install-static install-shared

# default distribution target
install: install-release-shared

clean:
	$(RM) -rf build bin lib

realclean: clean
	$(RM) -rf install

distclean: realclean
	$(RM) -rf ext


# Some alias targets to ease interactive use in command shells
configure-r: configure-release
configure-d: configure-debug
configure-rs: configure-release-shared
configure-ds: configure-debug-shared
configure-p: configure-relwithdebinfo
configure-ps: configure-relwithdebinfo-shared

clean-r: clean-release
clean-d: clean-debug
clean-rs: clean-release-shared
clean-ds: clean-debug-shared
clean-p: clean-relwithdebinfo
clean-ps: clean-relwithdebinfo-shared

install-r: install-release
install-d: install-debug
install-rs: install-release-shared
install-ds: install-debug-shared
install-p: install-relwithdebinfo
install-ps: install-relwithdebinfo-shared


ifeq ($(CMAKE_MCONFIG),)
TEST_TARGET := test
else
TEST_TARGET := RUN_TESTS
endif

# test target
test: release
	cmake $(BUILD_OPTIONS-release) --target $(TEST_TARGET) $(BUILD_TOOL_OPTIONS)

#
# project specific targets
#

# build the list of release, debug targets given the generic targets
TARGETS_RELEASE := $(foreach t,$(TARGETS),$(t)-r)
TARGETS_DEBUG := $(foreach t,$(TARGETS),$(t)-d)
TARGETS_RELWITHDEBINFO := $(foreach t,$(TARGETS),$(t)-p)

TARGETS_RELEASE_CLEAN_FIRST := $(foreach t,$(TARGETS),$(t)-cr)
TARGETS_DEBUG_CLEAN_FIRST := $(foreach t,$(TARGETS),$(t)-cd)
TARGETS_RELWITHDEBINFO_CLEAN_FIRST := $(foreach t,$(TARGETS),$(t)-cp)

$(TARGETS_RELEASE): $(BUILD_DIR-release)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-release) $(BUILD_JOBS) --target $(patsubst %-r,%,$@) $(BUILD_TOOL_OPTIONS)

$(TARGETS_RELEASE_CLEAN_FIRST): $(BUILD_DIR-release)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-release) $(BUILD_JOBS) --clean-first --target $(patsubst %-cr,%,$@) $(BUILD_TOOL_OPTIONS)

$(TARGETS_DEBUG): $(BUILD_DIR-debug)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-debug) $(BUILD_JOBS) --target $(patsubst %-d,%,$@) $(BUILD_TOOL_OPTIONS)

$(TARGETS_DEBUG_CLEAN_FIRST): $(BUILD_DIR-debug)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-debug) $(BUILD_JOBS) --clean-first --target $(patsubst %-cd,%,$@) $(BUILD_TOOL_OPTIONS)

$(TARGETS_RELWITHDEBINFO): $(BUILD_DIR-relwithdebinfo)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-relwithdebinfo) $(BUILD_JOBS) --target $(patsubst %-p,%,$@) $(BUILD_TOOL_OPTIONS)

$(TARGETS_RELWITHDEBINFO_CLEAN_FIRST): $(BUILD_DIR-relwithdebinfo)/CMakeCache.txt
	cmake $(BUILD_OPTIONS-relwithdebinfo) $(BUILD_JOBS) --clean-first --target $(patsubst %-cp,%,$@) $(BUILD_TOOL_OPTIONS)

.PHONY: install clean realclean distclean

.NOTPARALLEL:
