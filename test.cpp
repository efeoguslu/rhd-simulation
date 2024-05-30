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
    // Test 1: Sequence starts at 0, rises to 1, falls to -1, then stabilizes at 0
    std::deque<double> states1 = {0, 0, 0, 0, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0};
    std::cout << "Test 1: Sequence starts at 0, rises to 1, falls to -1, then stabilizes at 0\n";
    std::cout << "Expected: Falling (1)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states1)) << "\n\n";

    // Test 2: Sequence starts at 0, falls to -1, rises to 1, then stabilizes at 0
    std::deque<double> states2 = {0, 0, 0, 0, 0, -1, -1, -1, -1, -1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
    std::cout << "Test 2: Sequence starts at 0, falls to -1, rises to 1, then stabilizes at 0\n";
    std::cout << "Expected: Rising (0)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states2)) << "\n\n";

    // Test 3: Sequence starts at 0, rises to 1, and remains at 1
    std::deque<double> states3 = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
    std::cout << "Test 3: Sequence starts at 0, rises to 1, and remains at 1\n";
    std::cout << "Expected: Rising (0)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states3)) << "\n\n";

    // Test 4: Sequence starts at 0, falls to -1, then stabilizes at 0
    std::deque<double> states4 = {0, 0, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0};
    std::cout << "Test 4: Sequence starts at 0, falls to -1, then stabilizes at 0\n";
    std::cout << "Expected: Falling (1)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states4)) << "\n\n";

    // Test 5: Sequence consists entirely of 0s
    std::deque<double> states5 = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::cout << "Test 5: Sequence consists entirely of 0s\n";
    std::cout << "Expected: Stable (2)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states5)) << "\n\n";

    
    std::cout << "\n-------------------------------------\n";

    // Test 6: Single state transition from 0 to 1 then back to 0
    std::deque<double> states6 = {0, 1, 0};
    std::cout << "Test 6: Single state transition from 0 to 1 then back to 0\n";
    std::cout << "Expected: Rising (0)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states6)) << "\n\n";

    // Test 7: Single state transition from 0 to -1 then back to 0
    std::deque<double> states7 = {0, -1, 0};
    std::cout << "Test 7: Single state transition from 0 to -1 then back to 0\n";
    std::cout << "Expected: Falling (1)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states7)) << "\n\n";

    // Test 8: Multiple transitions between states
    std::deque<double> states8 = {0, 1, 0, -1, 0, 1, 0};
    std::cout << "Test 8: Multiple transitions between states\n";
    std::cout << "Expected: Rising (0), due to the last transition\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states8)) << "\n\n";

    // Test 9: No transitions, but initial state is not 0
    std::deque<double> states9 = {1, 1, 1, 1, 1};
    std::cout << "Test 9: No transitions, but initial state is not 0\n";
    std::cout << "Expected: Stable (2)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states9)) << "\n\n";

    // Test 10: Empty sequence
    std::deque<double> states10;
    std::cout << "Test 10: Empty sequence\n";
    std::cout << "Expected: Stable (2)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states10)) << "\n\n";

    // Test 11: Single element sequence
    std::deque<double> states11 = {0};
    std::cout << "Test 11: Single element sequence\n";
    std::cout << "Expected: Stable (2)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states11)) << "\n\n";

    // Test 12: Transition from 0 to 1, stays at 1
    std::deque<double> states12 = {0, 1, 1, 1};
    std::cout << "Test 12: Transition from 0 to 1, stays at 1\n";
    std::cout << "Expected: Rising (0)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states12)) << "\n\n";

    // Test 13: Transition from 0 to -1, stays at -1
    std::deque<double> states13 = {0, -1, -1, -1};
    std::cout << "Test 13: Transition from 0 to -1, stays at -1\n";
    std::cout << "Expected: Falling (1)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states13)) << "\n\n";

    // Test 14: Starts at 1, goes to 0, then to -1
    std::deque<double> states14 = {1, 0, -1};
    std::cout << "Test 14: Starts at 1, goes to 0, then to -1\n";
    std::cout << "Expected: Falling (1)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states14)) << "\n\n";

    // Test 15: Starts at -1, goes to 0, then to 1
    std::deque<double> states15 = {-1, 0, 1};
    std::cout << "Test 15: Starts at -1, goes to 0, then to 1\n";
    std::cout << "Expected: Rising (0)\n";
    std::cout << "Result: " << static_cast<int>(getStateChange(states15)) << "\n\n";

    return 0;
}