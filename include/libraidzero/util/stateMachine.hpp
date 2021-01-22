#pragma once

#include "libraidzero/util/taskWrapper.hpp"

/**
 * State machine helper class.
 *
 * @tparam States       An enum class representing the states of a subsystem. Required to have an
 *                      off state.
 * @tparam assumedState Optional - The assumed last state when using setNewState. Initially calling
 *                      setNewState with this state will not trigger a state transition.
 */
template <typename States, States assumedState = States::off>
class StateMachine : public TaskWrapper {

public:
    StateMachine() = default;
    ~StateMachine() override = default;

    /**
     * Sets the state.
     *
     * @param istate The state
     */
    virtual void setState(const States &istate) {
        state = istate;
    }

    /**
     * Sets the state and waits until the statemachine reports done.
     *
     * @param istate The istate
     */
    virtual void setStateBlocking(const States &istate) {
        _isDone = false;
        state = istate;
        while (!isDone()) {
            pros::delay(20);
        };
    }

    /**
     * Sets the state only if the state is different from the last time this function was called.
     *
     * @param istate The state
     */
    virtual void setNewState(const States &istate) {
        if (istate != lastState) {
            state = istate;
            lastState = istate;
        }
    }

    /**
     * Gets the state.
     *
     * @return The state.
     */
    virtual const States &getState() const {
        return state;
    }

    /**
     * Gets the state.
     *
     * @return The state.
     */
    virtual bool isDone() const {
        return _isDone;
    }

protected:
    /**
     * Override this method to implement setup procedures.
     */
    virtual void initialize() = 0;

    /**
     * Override this method to implement the statemachine task
     */
    void loop() override = 0;

    virtual void setDone() {
        _isDone = true;
    }

    States state{States::off};
    States lastState{assumedState};

    bool _isDone = false;
};
