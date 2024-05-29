#include <deque>
#include <iostream>

enum class SequenceType {
    Rising,
    Falling,
    Stable
};

SequenceType getStateChange(const std::deque<double>& states) {
    if (states.size() < 2) {
        return SequenceType::Stable;
    }

    enum State { STABLE, RISING, FALLING, UNKNOWN } currentState = STABLE;

    for (size_t i = 1; i < states.size(); ++i) {
        if (currentState == STABLE) {
            if (states[i-1] == 0) {
                if (states[i] == 1) {
                    currentState = RISING;
                } else if (states[i] == -1) {
                    currentState = FALLING;
                }
            }
        } else if (currentState == RISING) {
            if (states[i-1] == 1) {
                if (states[i] == 0) {
                    return SequenceType::Rising;
                } else if (states[i] == -1) {
                    currentState = FALLING;
                }
            }
        } else if (currentState == FALLING) {
            if (states[i-1] == -1) {
                if (states[i] == 0) {
                    return SequenceType::Falling;
                } else if (states[i] == 1) {
                    currentState = RISING;
                }
            }
        }
    }

    return SequenceType::Stable;
}

int main() {
    std::deque<double> states1 = {0, 0, 0, 0, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0};
    std::deque<double> states2 = {0, 0, 0, 0, 0, -1, -1, -1, -1, -1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
    std::deque<double> states3 = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
    std::deque<double> states4 = {0, 0, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0};
    std::deque<double> states5 = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    std::cout << static_cast<int>(getStateChange(states1)) << std::endl; // Should output Falling (1)
    std::cout << static_cast<int>(getStateChange(states2)) << std::endl; // Should output Rising (0)
    std::cout << static_cast<int>(getStateChange(states3)) << std::endl; // Should output Rising (0)
    std::cout << static_cast<int>(getStateChange(states4)) << std::endl; // Should output Falling (1)


    std::cout << static_cast<int>(getStateChange(states5)) << std::endl; // Should output Stable (2)

    return 0;
}