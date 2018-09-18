//
// Created by aadc on 10.09.18.
//

#include "PixelMetricTransformer.h"

PixelMetricTransformer::PixelMetricTransformer(float t_matrix[2][3]) {
    matrix = Matx23f(t_matrix[0][0], t_matrix[0][1], t_matrix[0][2], t_matrix[1][0], t_matrix[1][1], t_matrix[1][2]);
    invertAffineTransform(matrix, inverted);
}

float *PixelMetricTransformer::toMeter(float x, float y) {
    vector<Point2f> vec;
    vec.push_back(Point2f(x,y));

    transform(vec, vec, matrix);
    static float result[2] = { vec.front().x, vec.front().y };

    return result;
}

float *PixelMetricTransformer::toPixel(float x, float y) {
    vector<Point2f> vec;
    vec.push_back(Point2f(x,y));

    transform(vec, vec, inverted);
    static float result[2] = { vec.front().x, vec.front().y };

    return result;
}
