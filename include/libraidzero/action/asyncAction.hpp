#pragma once

#include <functional>

class AsyncAction {
public:
    double unitsError = 0.0;
    int timeFromStart = 0;
    std::function<void()> callback;

    AsyncAction(std::function<void()> icallback,
        double iunitsError, int itimeFromStart = -1
    );
};