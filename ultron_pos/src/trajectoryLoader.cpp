#include "trajectoryLoader.hpp"

int TrajectoryLoader::LoadMatrix(std::string fileName,Eigen::MatrixXd& matrix) {
  std::vector<double> matrixEntries;
  std::ifstream matrixDataFile(fileName);
  if(matrixDataFile.fail()){
    return 1;
  }
  std::string matrixRowString;
  std::string matrixEntry;
  int matrixRowNumber = 0;

  while (getline(matrixDataFile, matrixRowString))  // here we read a row by row of matrixDataFile and store every line
                                                    // into the string variable matrixRowString
  {
    std::stringstream matrixRowStringStream(
        matrixRowString);  // convert matrixRowString that is a string to a stream variable.

    while (getline(matrixRowStringStream, matrixEntry,
                   ','))  // here we read pieces of the stream matrixRowStringStream until every comma, and store the
                          // resulting character into the matrixEntry
    {
      matrixEntries.push_back(stod(matrixEntry));  // here we convert the string to double and fill in the row vector
                                                   // storing all the matrix entries
    }
    matrixRowNumber++;  // update the column numbers
  }
  matrix =  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
  return 0;
}

Eigen::MatrixXd TrajectoryLoader::GetArmStateTrajectory() const { return stateTrajectory_.bottomRows(6); }

Eigen::MatrixXd TrajectoryLoader::GetArmVelTrajectory() const { return velTrajectory_.bottomRows(6); }

Vector6d TrajectoryLoader::GetArmStateAtTime(double time) const {
  int index = std::floor(time / timeStep_);
  double delta = time - index * timeStep_;
  Vector6d q;
  q.setZero();
  q = stateTrajectory_.bottomRows(6).col(index) +
      delta / timeStep_ * (stateTrajectory_.bottomRows(6).col(index + 1) - stateTrajectory_.bottomRows(6).col(index));
  return q;
}

Vector6d TrajectoryLoader::GetArmVelAtTime(double time) const {
  int index = std::floor(time / timeStep_);
  double delta = time - index * timeStep_;
  Vector6d v;
  v.setZero();
  v = velTrajectory_.bottomRows(6).col(index) +
      delta / timeStep_ * (velTrajectory_.bottomRows(6).col(index + 1) - velTrajectory_.bottomRows(6).col(index));
  return v;
}