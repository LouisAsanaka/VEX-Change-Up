#include "libraidzero/planner/hermiteInterpolator.hpp"

#include <stdexcept>
#include <cmath>
#include <iostream>
#include <functional>

void planner::HermiteInterpolator::addSamplePoint(double x, const std::vector<double>& value) {
    if (value.size() > 20) {
        throw std::invalid_argument("too many derivatives");
    }

    for (std::size_t i = 0; i < value.size(); ++i) {
        std::vector<double> y;
        y.push_back(value[i]);

        if (i > 1) {
            double inv = 1.0 / FACTORIALS[i];
            for (std::size_t j = 0; j < y.size(); ++j) {
                y[j] *= inv;
            }
        }

        // update the bottom diagonal of the divided differences array
        int n = abscissae.size();
        bottomDiagonal.insert(bottomDiagonal.begin() + (n - i), y);

        auto bottom0 = std::ref(y);
        for (int j = i; j < n; ++j) {
            std::vector<double>& bottom1 = bottomDiagonal[n - (j + 1)];
            double inv = 1.0 / (x - abscissae[n - (j + 1)]);
            if (std::isinf(inv)) {
                throw std::invalid_argument("division by zero due to duplicated x-value");
            }
            for (int k = 0; k < y.size(); ++k) {
                bottom1[k] = inv * (bottom0.get()[k] - bottom1[k]);
            }
            bottom0 = std::ref(bottom1);
        }

        // update the top diagonal of the divided differences array
        std::vector<double> newVec = bottom0.get();
        topDiagonal.push_back(newVec);

        // update the abscissae array
        abscissae.push_back(x);
    }
}

std::vector<planner::PolynomialFunction> planner::HermiteInterpolator::getPolynomials() const {
    // iteration initialization
    const PolynomialFunction zero{std::vector<double>{0}};

    int size = topDiagonal[0].size();
    std::vector<planner::PolynomialFunction> polynomials{};
    polynomials.reserve(size);
    for (int i = 0; i < size; ++i) {
        polynomials.push_back(zero);
    }
    PolynomialFunction coeff{std::vector<double>{1}};

    // build the polynomials by iterating on the top diagonal of the divided differences array
    for (int i = 0; i < topDiagonal.size(); ++i) {
        const std::vector<double>& tdi = topDiagonal[i];
        for (int k = 0; k < polynomials.size(); ++k) {
            polynomials[k] = polynomials[k].add(coeff.multiply(
                PolynomialFunction{std::vector<double>{tdi[k]}}));
        }
        coeff = coeff.multiply(PolynomialFunction{
            std::vector<double>{-abscissae[i], 1.0}
        });
    }

    return polynomials;
}

std::vector<double> planner::HermiteInterpolator::value(double x) const {
    std::vector<double> value(topDiagonal[0].size());
    double valueCoeff = 1;
    for (int i = 0; i < topDiagonal.size(); ++i) {
        std::vector<double> dividedDifference = topDiagonal[i];
        for (int k = 0; k < value.size(); ++k) {
            value[k] += dividedDifference[k] * valueCoeff;
        }
        double deltaX = x - abscissae[i];
        valueCoeff *= deltaX;
    }
    return value;
}
