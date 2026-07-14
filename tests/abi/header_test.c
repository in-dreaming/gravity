#include "gravity.h"

#if GRAVITY_V1_ABI_VERSION != 1
#error "unexpected ABI version"
#endif

#if GRAVITY_V1_PROTOCOL_VERSION != 1
#error "unexpected protocol version"
#endif

int main(void) {
    return (GRAVITY_V1_SNAPSHOT_FORMAT_VERSION == 1 &&
            GRAVITY_V1_ASSET_FORMAT_VERSION == 1) ? 0 : 1;
}
