#ifndef _RNEA
#define _RNEA

#include "Control.h"

typedef struct { double v[3][3]; 	}sMatrix3;
typedef struct { double v[4][4]; 	}sMatrix4;
typedef struct { double v[6]; 		}sVector6;
typedef struct { double v[4]; 		}sVector4;
typedef struct { double v[3]; 		}sVector3;

void RNEA_Conf(void);
void RNEA_CalcTorques(void);

#endif
