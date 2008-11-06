/* Import (extern) header for application - AppBuilder 2.03  */

#include "abdefine.h"

extern ApWindowLink_t base;
extern ApWidget_t AbWidgets[ 3 ];


#ifdef __cplusplus
extern "C" {
#endif
int RawActivate( PtWidget_t *widget, ApInfo_t *data, PtCallbackInfo_t *cbinfo );
void dibujo( PtWidget_t *widget, PhTile_t *damage ) 

;
int raw_init( PtWidget_t *widget ) 

;
#ifdef __cplusplus
}
#endif
