#include "libraidzero/action/asyncAction.hpp"

AsyncAction::AsyncAction(std::function<void()> icallback,
    double iunitsError, int itimeFromStart
) : callback{std::move(icallback)},
    unitsError{iunitsError}, 
    timeFromStart{itimeFromStart}
{

}