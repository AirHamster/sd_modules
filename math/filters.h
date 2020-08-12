/**
 * @file    filter.h
 * @brief   Filters funcs.
 *
 * @addtogroup MATH
 * @{
 */

#ifndef FILTERS_H
#define FILTERS_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/// floble = floating-point arithmetic type
/// float or double
#define floble float

#define INLINE static inline

/// A comparator function used by qsort 
 static inline int compare(const void *a, const void *b){
    return (*(int*) a) - (*(int*) b);
} 

INLINE floble getMean(floble *data, int length){
    int i = 0;
    
    floble mean = 0.;
    
    for(i = 0; i < length; ++i){
        mean += data[i];
    }
    
    return mean / ((floble) length);
}

INLINE floble getTrimmedMean(floble *data, int length, floble accuracy){
    int i = 0;
    int indexOfMin = 0;
    int indexOfMax = 0;
    int toBeFiltered = round(length * accuracy);
    
    while(0 < toBeFiltered){
        indexOfMin = 0;
        indexOfMax = 0;
        
        for(i = 0; i < length; ++i){
            if(data[i] < data[indexOfMin]){
                indexOfMin = i;
            }else{
                if(data[i] > data[indexOfMax]){
                    indexOfMax = i;
                }
            }
        }
        
        if(length - 1 != indexOfMax){
            data[indexOfMin] = data[length - 1];
        }else{
            data[indexOfMin] = data[length - 2];
        }
        
        data[indexOfMax] = data[length - 2];
        
        length -= 2;
        
        --toBeFiltered;
    }
    
    return getMean(data, length);
}

/// Better to implement  partial sort or use the C++ implementation
/// Be careful: array is modified
INLINE floble getMedianM(floble *data, int length){
    if(1 > length){
        return 0.;
    }

    int middle = length / 2;
    
    qsort(data, length, sizeof(data[0]), compare);
    
    if(0 == length % 2){
        return 0.5 * (data[middle - 1] + data[middle]);
    }else{
        return data[middle];
    }
}

INLINE void trimmedMeanFilter(
    floble *originalData,
    floble *filteredData,
    int length,
    int halfWindowSize,
    floble accuracy
){
    int i = 0;
    int j = 0;
    int windowSize = 2 * halfWindowSize + 1;
    
    floble *window;
    
    window = (floble *) calloc(windowSize, sizeof(floble));
    
    if(NULL == window){
        /*fprintf(
            stderr,
            "Error: allocation memory fail [calloc]. Time: %lld. File: %s@%d.\n",
            (long long) time(NULL),
            __FILE__,
            __LINE__
        );*/
        
        return;
    }
    
    for(i = 0; i < length; ++i){
        for(j = 0; j < windowSize; ++j){
            if(i + j < halfWindowSize){
                window[j] = originalData[0];
            }else{
                if(i + j - halfWindowSize >= length){
                    window[j] = originalData[length - 1];
                }else{
                    window[j] = originalData[i + j - halfWindowSize];
                }
            }
        }
        
        filteredData[i] = getTrimmedMean(window, windowSize, accuracy);
    }
    
    free(window);
}

INLINE void medianFilter(
    floble *originalData,
    floble *filteredData,
    int length,
    int halfWindowSize,
    floble accuracy
){
    int i = 0;
    int j = 0;
    int windowSize = 2 * halfWindowSize + 1;
    
    floble *window;
    
    window = (floble *) calloc(windowSize, sizeof(floble));
    
    if(NULL == window){
        /*fprintf(
            stderr,
            "Error: allocation memory fail [calloc]. Time: %lld. File: %s@%d.\n",
            (long long) time(NULL),
            __FILE__,
            __LINE__
        );*/
        
        return;
    }

    int partialWindowSize = 0;
    
    for(i = 0; i < length; ++i){
        partialWindowSize = 0;

        for(j = 0; j < windowSize; ++j){
            if(i + j < halfWindowSize || i + j - halfWindowSize >= length){
                continue;
            }else{
                window[partialWindowSize] = 
                    originalData[i + j - halfWindowSize];

                ++partialWindowSize;
            }
        }
        
        filteredData[i] = getMedianM(window, partialWindowSize);
    }
    
    free(window);
}
#endif
