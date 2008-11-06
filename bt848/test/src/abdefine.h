/* Define header for application - AppBuilder 2.03  */

/* 'base' Window link */
extern const int ABN_base;
#define ABW_base                             AbGetABW( ABN_base )
extern const int ABN_RawZone;
#define ABW_RawZone                          AbGetABW( ABN_RawZone )
extern const int ABN_Timer;
#define ABW_Timer                            AbGetABW( ABN_Timer )

#define AbGetABW( n ) ( AbWidgets[ n ].wgt )

#define AB_OPTIONS "s:x:y:h:w:S:"
