#include "bpa.h"



void bpa_unit_init(bpa_t *bpaUnit){
    for(int i=0; i<BPA_FOCAL_NUM; i++){
        bpaUnit->mass[i] = 0.0;
    }
}

int bpa_intersection_index_get(int index1,int index2){
    if(index1 == BPA_FOCAL_NUM){
        return index2;
    }
    if(index2 == BPA_FOCAL_NUM){
        return index1;
    }

    if(index1 == index2){
        return index1;
    }else{
        return -1;
    }
}


double bpa_combination_compute(const bpa_t *b1,const bpa_t *b2,bpa_t *result){
    double conflict = 0.0;
    double m_new[BPA_FOCAL_NUM] = {0.0};

    for (int i = 0; i < BPA_FOCAL_NUM; ++i) {
        for (int j = 0; j < BPA_FOCAL_NUM; ++j) {
            double mass_product = b1->mass[i] * b2->mass[j];
            if (mass_product == 0) continue;

            int intersection = bpa_intersection_index_get(i, j); // 鍋囪�炬湁涓€涓�鍑芥暟鍙�浠ヨ繑鍥炰氦闆嗗�瑰簲鐨勭储寮�
            if (intersection == -1) { // 濡傛灉鏄�鍐茬獊
                conflict += mass_product;
            } else {
                m_new[intersection] += mass_product;
            }
        }
    }

    double norm = 1.0 - conflict;
    if (norm <= 0) return -1; // 褰掍竴鍖栧け璐�

    for (int i = 0; i < BPA_FOCAL_NUM; ++i) {
        result->mass[i] = m_new[i] / norm;
    }

    return norm;
}

