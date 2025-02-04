/**********************************************************
                 			Clocks
**********************************************************/

#define MSEL    4               // PLL multiplier 	(1, 2, 3, o 4)
#define PCKDIV  1               // APB divider		(1, 2, o 4)

/********** Configuración PLL *********/
#if (MSEL==1)
	#define PSEL 4				// CCO=CCK*16
#else
	#if (MSEL==2)
		#define PSEL 3			// CCO=CCK*8
	#else
		#define PSEL 2			// CCO=CCK*4
	#endif
#endif

#define PLLCVAL (((PSEL-1)<<5)|(MSEL-1))

/********* Estados de espera en Flash *******/
#if (MSEL==4)
	#define FLASHWS 3
#else
	#define FLASHWS MSEL
#endif

