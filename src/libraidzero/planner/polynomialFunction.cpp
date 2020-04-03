#include "libraidzero/planner/polynomialFunction.hpp"

#include <stdexcept>
#include <vector>
#include <algorithm>

planner::PolynomialFunction::PolynomialFunction(const std::vector<double> c) : 
    coefficients{c}
{
}

double planner::PolynomialFunction::value(double x) const {
    return evaluate(coefficients, x);
}

double planner::PolynomialFunction::evaluate(
    const std::vector<double>& c, double argument) 
{
    int n = c.size();
    if (n == 0) {
        throw std::invalid_argument("empty coefficients");
    }
    double result = c[n - 1];
    for (int j = n - 2; j >= 0; j--) {
        result = argument * result + c[j];
    }
    return result;
}

std::vector<double> planner::PolynomialFunction::differentiate(const std::vector<double>& coefficients) {
    int n = coefficients.size();
    if (n == 0) {
        throw std::invalid_argument("empty coefficients");
    }
    if (n == 1) {
        return std::vector<double>{0};
    }
    std::vector<double> result(n - 1);
    for (int i = n - 1; i > 0; i--) {
        result[i - 1] = i * coefficients[i];
    }
    return result;
}

planner::PolynomialFunction planner::PolynomialFunction::add(const planner::PolynomialFunction& p) const {
    // identify the lowest degree polynomial
    int lowLength = std::min(coefficients.size(), p.coefficients.size());
    int highLength = std::max(coefficients.size(), p.coefficients.size());

    // build the coefficients array
    std::vector<double> newCoefficients(highLength);
    for (int i = 0; i < lowLength; ++i) {
        newCoefficients[i] = coefficients[i] + p.coefficients[i];
    }
    const std::vector<double>& source = (coefficients.size() < p.coefficients.size()) ?
        p.coefficients : coefficients;

    newCoefficients.insert(newCoefficients.begin() + lowLength, 
        source.begin() + lowLength, source.begin() + highLength);

    return PolynomialFunction(newCoefficients);
}

planner::PolynomialFunction planner::PolynomialFunction::multiply(const PolynomialFunction& p) const {
    std::vector<double> newCoefficients(coefficients.size() + p.coefficients.size() - 1);

    for (int i = 0; i < newCoefficients.size(); ++i) {
        newCoefficients[i] = 0.0;
        for (int j = std::max(0, (int)(i + 1 - p.coefficients.size()));
            j < std::min((int) coefficients.size(), i + 1);
            ++j) {
            newCoefficients[i] += coefficients[j] * p.coefficients[i - j];
        }
    }

    return PolynomialFunction(newCoefficients);
}

planner::PolynomialFunction planner::PolynomialFunction::derivative() const {
    return PolynomialFunction(differentiate(coefficients));
}
