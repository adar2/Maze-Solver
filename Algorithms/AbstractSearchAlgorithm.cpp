
#include "AbstractSearchAlgorithm.h"

double AbstractSearchAlgorithm::diff_clock(const clock_t &clock1, const clock_t &clock2) {
    double diff_ticks = double(clock1) - clock2;
    double diff_ms = (diff_ticks) / (double(CLOCKS_PER_SEC) / 1000);
    return diff_ms / 1000;

}

void AbstractSearchAlgorithm::generate_stats(const Node &current_node, double avg_heuristic_val) {
    std::string output_filename = getAlgorithmName() + '_' + getProblemName() + ".txt";
    std::ofstream outputFile(output_filename);
    outputFile << "Problem : " << getProblemName() << std::endl;
    outputFile << "total nodes explored: " << getExpanded() << std::endl;
    if (getEndStatus()) {
        auto path_start = current_node.getPathTilNow().begin();
        auto path_end = current_node.getPathTilNow().end();
        for (auto item = path_start + 1; item != path_end; ++item) {
            if (item->first == (item - 1)->first - 1 && item->second == (item - 1)->second - 1)
                outputFile << "LU" << '-';
            if (item->first == (item - 1)->first - 1 && item->second == (item - 1)->second)
                outputFile << "U" << '-';
            if (item->first == (item - 1)->first + 1 && item->second == (item - 1)->second - 1)
                outputFile << "LD" << '-';
            if (item->first == (item - 1)->first + 1 && item->second == (item - 1)->second)
                outputFile << "D" << '-';
            if (item->first == (item - 1)->first && item->second == (item - 1)->second + 1)
                outputFile << "R" << '-';
            if (item->first == (item - 1)->first && item->second == (item - 1)->second - 1)
                outputFile << "L" << '-';
            if (item->first == (item - 1)->first + 1 && item->second == (item - 1)->second + 1)
                outputFile << "RD" << '-';
            if (item->first == (item - 1)->first - 1 && item->second == (item - 1)->second + 1)
                outputFile << "RU" << '-';
        }
        outputFile << " solution cost: " << current_node.getActualCost();
        calcEBF(current_node.getDepth());
        calcDN(current_node.getDepth());
    } else {
        outputFile << "Failed";
        calcEBF(getMax());
        calcDN(getMax());
    }
    outputFile << std::endl;
    outputFile << "d/N : " << getDN() << std::endl;
    outputFile << "time in seconds: " << diff_clock(_current_time, _start_time) << std::endl;
    outputFile << "EBF : " << getEBF() << std::endl;
    outputFile << "Min cutoff : " << getMin() << std::endl;
    outputFile << "Max cutoff : " << getMax() << std::endl;
    outputFile << "Avg cutoff : " << getAvg() << std::endl;
    if (avg_heuristic_val > 0)
        outputFile << "Avg H Value: " << avg_heuristic_val << std::endl;
    outputFile.close();
}

double AbstractSearchAlgorithm::getAvg() const {
    if (_no_of_cutoffs == 0)
        return 0;
    return double(_sum_of_cutoffs_depths) / _no_of_cutoffs;
}

void AbstractSearchAlgorithm::addCutoffToSum(int cut_off_depth) {
    _sum_of_cutoffs_depths += cut_off_depth;
    ++_no_of_cutoffs;
}

void AbstractSearchAlgorithm::calcEBF(int depth) {
    if (!depth)
        return;
    setEBF(pow(getExpanded(), pow(depth, -1)));
}

void AbstractSearchAlgorithm::update_cutoffs(int cutoff_depth) {
    if (_min > cutoff_depth)
        _min = cutoff_depth;
    if (_max < cutoff_depth)
        _max = cutoff_depth;
    addCutoffToSum(cutoff_depth);
}

void AbstractSearchAlgorithm::calcDN(int depth) {
    if (!getExpanded())
        return;
    setDN(double(depth) / getExpanded());
}

void AbstractSearchAlgorithm::setProblemName(const std::string &problemName) {
    std::string tmp = problemName;
    tmp = tmp.substr(0, problemName.find(".txt"));
    tmp.erase(std::remove_if(tmp.begin(), tmp.end(), [](const char &c) { return !std::isalnum(c); }), tmp.end());
    _problem_name = tmp;
}

const std::string &AbstractSearchAlgorithm::getAlgorithmName() const {
    return _algorithm_name;
}

void AbstractSearchAlgorithm::setAlgorithmName(const std::string &algorithmName) {
    _algorithm_name = algorithmName;
}
