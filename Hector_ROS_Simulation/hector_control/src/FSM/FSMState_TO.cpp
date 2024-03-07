#include "../../include/FSM/FSMState_TO.h"

FSMState_TO::FSMState_TO(ControlFSMData *data)
                 :FSMState(data, FSMStateName::TO, "offlineTrajectory"){

                    std::string pathToData = ros::package::getPath("hector_control") + "/include/trajectory_data/example.csv";
                    try {
                        traj_ref = readDataFromFile(pathToData);
                        traj_data_loaded = true;
                    }catch (const std::runtime_error& e) {
                        std::cerr << e.what() << std::endl;
                        traj_data_loaded = false;
                        traj_ref = Eigen::MatrixXd::Zero(1, 34);
                    }
                 }

void FSMState_TO::enter()
{
         _data->_interface->zeroCmdPanel();
        counter = 0;
        _data->_desiredStateCommand->firstRun = true;
        _data->_stateEstimator->run(); 
        _data->_legController->zeroCommand();
}

void FSMState_TO::run()
{

    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run(); 
    _userValue = _data->_lowState->userValue;

    // Implement the trajectory logic and update the desired state commands here
    // Extract the trajectory at the current time

    // Set the desired state commands

    _data->_legController->updateCommand(_data->_lowCmd);  
    counter++;
}

void FSMState_TO::exit()
{      
    counter = 0; 
    _data->_interface->zeroCmdPanel();
}

FSMStateName FSMState_TO::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L2_B || !traj_data_loaded){
        std::cout << "\n\n******************************************************************************************" << std::endl;
        std::cout << "*******************************TO Transitioning to Passive********************************" << std::endl;
        std::cout << "Trajectory data loaded =  " << traj_data_loaded << std::endl;
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::TO;
    }
}

Eigen::MatrixXd FSMState_TO::readDataFromFile(const std::string& filePath) {
    std::vector<std::vector<double>> data;
    std::ifstream myFile(filePath);

    // If cannot open the file, report an error
    if (!myFile.is_open()) {
        throw std::runtime_error("\n Warning: Could not open file: " + filePath);
    }

    std::cout << "Reading optimization data from: " << filePath << std::endl;
    std::string line;

    // Read data line by line
    while (getline(myFile, line)) {
        std::stringstream ss(line);
        double val;
        std::vector<double> row;
        while (ss >> val) {
            row.push_back(val);
            if (ss.peek() == ',') ss.ignore();
        }
        data.push_back(row);
    }

    myFile.close();

    // Convert to Eigen type
    if (data.size() == 0) return Eigen::MatrixXd(); // Check for empty data
    size_t numRows = data.size();
    size_t numCols = data[0].size();
    Eigen::MatrixXd mat(numRows, numCols);
    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            mat(i, j) = data[i][j];
        }
    }

    return mat.transpose(); 
}


void FSMState_TO::extractTrajectory(const Eigen::MatrixXd& fullTrajectory, Eigen::VectorXd& currentTrajectory, double time, double dataFrequency) {

    int index = static_cast<int>(floor(time * dataFrequency/1000.0));
    
    // Check if the index is within the bounds of the trajectory matrix
    if (index < 0 || index >= fullTrajectory.rows()) {
        std::cerr << "Error: Time index is out of bounds" << std::endl;
        currentTrajectory = fullTrajectory.row(fullTrajectory.rows()-1).transpose(); //set track to last row of trajectory
        return;
    }
    currentTrajectory = fullTrajectory.row(index).transpose();
}