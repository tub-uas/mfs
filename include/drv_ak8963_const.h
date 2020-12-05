#ifndef DRV_AK8963_CONST_H
#define DRV_AK8963_CONST_H

// == SOFT IRON ================================================================
#define MAG_SI_11      86.8227209
#define MAG_SI_12      -0.2403698
#define MAG_SI_13       0.8596319
#define MAG_SI_21      -0.2403698
#define MAG_SI_22      96.2030783
#define MAG_SI_23      -0.4638596
#define MAG_SI_31       0.8596319
#define MAG_SI_32      -0.4638596
#define MAG_SI_33      88.4975581

// == HARD IRON ================================================================
// Mcorr = SI*(M-HI)
#define MAG_HI_1       15.8836179
#define MAG_HI_2       51.5660664
#define MAG_HI_3      -44.1729453

// == CURRENT SCALING ==========================================================
// MCScorr = Mcorr - CS*curr
#define MAG_CS_1        0.2485534
#define MAG_CS_2        0.3516560
#define MAG_CS_3       -0.2448827

#endif // DRV_AK8963_CONST_H
