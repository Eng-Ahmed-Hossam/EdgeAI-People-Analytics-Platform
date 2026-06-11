#include "person_count.h"

int person_count(const Detection *dets, int n)
{
    int cnt = 0;
    for (int i = 0; i < n; i++) {
        if (dets[i].class_id == 0)
            cnt++;
    }
    return cnt;
}
