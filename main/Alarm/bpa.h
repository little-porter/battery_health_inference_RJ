#ifndef BPA_H
#define BPA_H


#define BPA_FOCAL_NUM   10

typedef enum bpa_focal{
    BPA_FOCAL_FIRE      = 0,
    BPA_FOCAL_NOFIRE    = 1,
    BPA_FOCAL_FAULT     = 2,
}bpa_focal_t;



typedef struct _bpa{
    int focal_elements[BPA_FOCAL_NUM];      // 元素
    double mass[BPA_FOCAL_NUM];             // 元素权重
} bpa_t;

void bpa_unit_init(bpa_t *bpaUnit);
double bpa_combination_compute(const bpa_t *b1,const bpa_t *b2,bpa_t *result);

#endif
