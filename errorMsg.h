//to do: log error to sd card?

class ErrorManager {
private:
    std::unordered_map<std::string, unsigned long> errorMap; // Map of errors
    unsigned long errorVisibilityDuration = 60000;   // 30 seconds

public:
    // Add or update an error
    void reportError(const std::string& errorMessage, unsigned long currentTime) {
        errorMap[errorMessage] = currentTime;
    }
    
    // check if errors existing
    bool checkErrorActive() {
        if (errorMap.empty()) {
            return false;
        } else {
            return true;
        }
    }

    // Clear inactive errors that exceeded visibility duration
    void clearExpiredErrors(unsigned long currentTime) {
        for (auto it = errorMap.begin(); it != errorMap.end(); ) {
            if (currentTime - it->second > errorVisibilityDuration) {
                it = errorMap.erase(it);  // Erase and move to the next element
            } else {
                ++it;  // Only increment if we didn't erase
            }
        }
    }

    // Display all active errors
    std::string getErrorMsgs() {
        std::string allErrorMsgs = "";
        for (auto errors : errorMap) {
            allErrorMsgs = allErrorMsgs + " & " + errors.first;
        }
        return allErrorMsgs;
    }
};