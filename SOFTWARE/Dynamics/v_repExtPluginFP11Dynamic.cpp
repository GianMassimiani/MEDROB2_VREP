#include "v_repExtPluginSkeleton.h"
#include "luaFunctionData.h"
#include "v_repLib.h"
#include <Eigen/LU>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <string>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////GLOBAL VARIABLES DECLARATION/////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

simInt ui_handle;
//types of trajectory (straight line, circular, spline)
int traj = 1;
//types of task (trajectory tracking via error linearization, .... , posture regulation, ...)
int task = 1;
//localization types (ideal or odometric)
int loc = 1;
//robot parameters
double robot_height = 0.0975;
double radius;
double xc;
double yc;
simFloat v = 0;
simFloat w = 0;
double w_l, w_r;
double wheel_radius;
double axel_length;
simFloat rwheel_pos;
simFloat lwheel_pos;
simFloat past_lw_pos;
simFloat past_rw_pos;
simInt robot;
simInt ref_point;
simInt b_point;
simInt motorLeft, motorRight;
// control parameters
int gain_mod = 1;
double b_length = 0.1;
simFloat b_pos[3] = {0,0,0};
double k1, k2, k3, k4;
double k_gains[4] = {0,0,0,0};
double rho, gam, delta;
//Interface parameters
double param[10];
//Simulation parameters
simFloat time_step;
double T = 10.0;
const double pi = 3.14159;
simFloat start_point[3] = {0,0,0};
simFloat end_point[3] = {0,0,0};
simFloat finalPos[3] = {0,0,0};
simFloat finalOrient[3] = {0,0,0};
simChar* finalConf = "";
simFloat currentConf[3], desiredConf[3],past_pose[3],real_pose[3];
simFloat localization_error[3]={0,0,0};
simInt removable_handlers[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
simFloat v_old;
//obstacles
simInt tbox;
bool obstacle_flag=false;
//tracking parameters
simInt tracking_path;
simInt tracking_point_number = 0;
int tracking_numb = 0;
simInt path_handle, path_dummy;
//Spline settings
simFloat l1, l2, l3, l4;
simFloat L;
Eigen::VectorXf vx, vy;
double spline_t1, spline_t2, spline_t3, spline_t4;
double spline_tin, spline_tfin, spline_lin, spline_lfin;
//graph parameters
simInt error_graph_handler;
simInt trajectory_graph_handler;

//function declaration
void create_linear_traj();
void create_circular_traj();
void create_spline_traj();
void compute_desired_position(double);
void check_values(int, int, double*, double);
double  bound_angle_error(double e);
void linear_controller();
void non_linear_controller();
void static_io_linearization();
void dynamic_io_linearization(double);
void cartesian_reg();
void posture_reg();
void stopSim();
void tracking(simInt);
void PlotTrajectoryGraph();
void CreateObstacles();
Eigen::MatrixXf create_matrix();

//data file
std::ofstream data_file;

#ifdef _WIN32
#ifdef QT_COMPIL
#include <direct.h>
#else
#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#endif
#endif
#if defined (__linux) || defined (__APPLE__)
#include <unistd.h>
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

#define PLUGIN_VERSION 2

LIBRARY vrepLib;
#define LUA_GETSENSORDATA_COMMAND "simExtSkeleton_getSensorData"

const int inArgs_GETSENSORDATA[]={
    3,
    sim_lua_arg_int,0,
    sim_lua_arg_float|sim_lua_arg_table,3,
    sim_lua_arg_int|sim_lua_arg_table,2, };

void LUA_GETSENSORDATA_CALLBACK(SLuaCallBack* p)
{ 	p->outputArgCount=0;
    CLuaFunctionData D;
    bool commandWasSuccessful=false;
    int returnResult;
    std::vector<float> returnData;
    float returnDistance;
    if (D.readDataFromLua(p,inArgs_GETSENSORDATA,inArgs_GETSENSORDATA[0],LUA_GETSENSORDATA_COMMAND))
    { 	std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        
        int sensorIndex=inData->at(0).intData[0];
        std::vector<float>& floatParameters=inData->at(1).floatData;
        std::vector<int>& intParameters=inData->at(2).intData;
        
        
        if ((sensorIndex>=0)&&(sensorIndex<10))
        {
            commandWasSuccessful=true;
            returnResult=1;
            returnData.push_back(1.0f);
            returnData.push_back(2.0f);
            returnData.push_back(3.0f);
            returnDistance=59.0f;
        }
        else
            simSetLastError(LUA_GETSENSORDATA_COMMAND,"Invalid sensor index."); // output an error message to the simulator's status bar
    }
    if (commandWasSuccessful)
    {
        D.pushOutData(CLuaFunctionDataItem(returnResult));
        D.pushOutData(CLuaFunctionDataItem(returnData));
        D.pushOutData(CLuaFunctionDataItem(returnDistance));
    }
    D.writeDataToLua(p);
}

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
    
    char curDirAndFile[1024];
#ifdef _WIN32
#ifdef QT_COMPIL
    _getcwd(curDirAndFile, sizeof(curDirAndFile));
#else
    GetModuleFileName(NULL,curDirAndFile,1023);
    PathRemoveFileSpec(curDirAndFile);
#endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif
    
    std::string currentDirAndPath(curDirAndFile);
    
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp+="\\v_rep.dll";
#elif defined (__linux)
    temp+="/libv_rep.so";
#elif defined (__APPLE__)
    temp+="/libv_rep.dylib";
#endif
    vrepLib=loadVrepLibrary(temp.c_str());
    if (vrepLib==NULL)
    {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        return(0);
    }
    if (getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0);
    }
    
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
    if (vrepVer<30200)
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); 	}
    
    
    std::vector<int> inArgs;
    
    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETSENSORDATA,inArgs);
    simRegisterCustomLuaFunction(LUA_GETSENSORDATA_COMMAND,strConCat("number result,table data,number distance=",LUA_GETSENSORDATA_COMMAND,"(number sensorIndex,table_3 floatParameters,table_2 intParameters)"),&inArgs[0],LUA_GETSENSORDATA_CALLBACK);
    
    return(PLUGIN_VERSION); }


VREP_DLLEXPORT void v_repEnd()
{
    
    unloadVrepLibrary(vrepLib);
}


VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
    static bool refreshDlgFlag=true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    void* retVal=NULL;
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////INTERFACE SETTINGS AND SIGNAL RETRIEVAL//////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //The signals sent from the user interface are retrieved and the corresponding variables are initialized
    
    ui_handle = simGetUIHandle("UI");
    
    simInt aVal[2];
    aVal[0] = NULL;
    aVal[1] = NULL;
    simInt button_handle = simGetUIEventButton(ui_handle, aVal);
    
    //TASK TYPE:
    //task1 = trajectory tracking via error linearization
    //task2 = trajectory tracking via non linear control
    //task3 = trajectory tracking via dynamic I/O linearization
    //task4 = trajectory tracking via static I/O linearization
    //task5 = cartesian regulation
    //task6 = posture regulation
    if (button_handle >= 5 && button_handle <=10){
        simChar* label = simGetUIButtonLabel(ui_handle, button_handle);
        switch(button_handle){
            case 5:
                task = 1;
                break;
            case 6:
                task = 2;
                break;
            case 7:
                task = 3;
                break;
            case 8:
                task = 4;
                break;
            case 9:
                task = 5;
                traj = 0;
                simSetUIButtonLabel(ui_handle, 34, " ", NULL);
                break;
            case 10:
                task = 6;
                traj = 0;
                simSetUIButtonLabel(ui_handle, 34, " ", NULL);
                break;
            default: task = 0;
        }
        simSetUIButtonLabel(ui_handle, 32,label, NULL);
    }
    
    //trajectory type: traj1 = linear trajectory; traj2 = circular trajectory; traj3 = spline trajectory
    if (button_handle == 11 ||button_handle == 16 ||button_handle == 20){
        simChar* label = simGetUIButtonLabel(ui_handle, button_handle);
        switch(button_handle){
            case 11:
                traj = 1;
                break;
            case 16:
                traj = 2;
                break;
            case 20:
                traj = 3;
                break;
            default: traj = 0;
        }
        simSetUIButtonLabel(ui_handle, 34,label, NULL);
    }
    
    //localization type: loc1 = ideal localization; loc2 = odometric localization
    if (button_handle == 36 ||button_handle == 37 ){
        simChar* label = simGetUIButtonLabel(ui_handle, button_handle);
        switch(button_handle){
            case 36:
                loc = 1;
                break;
            case 37:
                loc = 2;
                break;
            default: loc = 0;
        }
        simSetUIButtonLabel(ui_handle, 38,label, NULL);
    }
    
    //gain modality: mod1 = gain expressed through k parameters; mod2 = gain expressed through 'a' and 'chi' (for error linearization)
    if (button_handle == 42 || button_handle == 49){
        switch(button_handle){
            case 42:
                gain_mod = 1;
                break;
            case 49:
                gain_mod = 2;
                break;
            default:;
        }
    }
    
    
    //obstacles activated or deactivated
    if (button_handle == 52){
        simChar* label = simGetUIButtonLabel(ui_handle, button_handle);
        obstacle_flag= true;
        simSetUIButtonLabel(ui_handle, 54,label, NULL);
    }
    if (button_handle == 53){
        simChar* label = simGetUIButtonLabel(ui_handle, button_handle);
        obstacle_flag= false;
        simSetUIButtonLabel(ui_handle, 54,label, NULL);
    }
    
    
    //start simulation:
    //in base of the selected task the corresponding parameters are read, checked (they have to be consistent, otherwise default values are given) and stored.
    if (button_handle == 39){
        
        if (simGetSimulationState() == sim_simulation_advancing_running){
            simStopSimulation();
        }
        
        
        
        if ( task >= 1 && task <= 4 ){
            
            switch(traj){
                case 1:
                    check_values(12,15, param, 0.0);
                    break;
                case 2:
                    check_values(17,19, param, 0.0);
                    break;
                case 3:
                    check_values(21,30, param, 0.0);
                    break;
                default:;
            }
            if (task == 4){
                // check the consistency of the b_length value
                check_values(41,41, &b_length, 0.01);
            }
            else{
                // if we are not in the I/O linearization, we set the b_length to 0.01
                b_length = 0.01;
            }
            check_values(50,50, &T, 10.0);
            for (int i = 0; i < 10; i++){
                if (removable_handlers[i] != -1){
                    simRemoveObject(removable_handlers[i]);
                    removable_handlers[i] = -1;
                }
            }
            
        }
        
        if (gain_mod == 1){
            check_values(43,46, k_gains, 1.0);
        }else {
            check_values(47,48, k_gains, 1.0);
        }
        
        simStartSimulation();
    }
    
    
    
    
    
    
    if (message==sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag=true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too
    
    if (message==sim_message_eventcallback_menuitemselected)
    { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }
    
    if (message==sim_message_eventcallback_instancepass)
    {	// This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:
        
        int flags=auxiliaryData[0];
        bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
        bool instanceSwitched=((flags&64)!=0);
        
        
        if (instanceSwitched)
        {
            // React to an instance switch here!!
        }
        
        if (sceneContentChanged)
        { // we actualize plugin objects for changes in the scene
            
            //...
            
            refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
        
        
        
        
        
        
        //**************************************************************************************************************//
        //******************************    ADVANCING RUNNING    *******************************************************//
        //**************************************************************************************************************//
        if (simGetSimulationState() == sim_simulation_advancing_running){
            double time = simGetSimulationTime();
            
            
            //create obstacles if desired
            if (time < 0.1 && obstacle_flag==true){
                CreateObstacles();
            }
            
            
            // COMPUTE CURRENT CONFIGURATION AND B_POINT POSITION
            if (loc == 1){
                // ideal
                simFloat tmp[3] = {0,0,0};
                simGetObjectPosition(ref_point, -1, tmp);
                currentConf[0] = tmp[0];
                currentConf[1] = tmp[1];
                simGetObjectOrientation(ref_point, -1, tmp);
                currentConf[2] = tmp[2];
                //if task is static io linearization (task ==4) update the position of the b_point
                if (task == 4){
                    simGetObjectPosition(b_point, -1, tmp);
                    b_pos[0] = tmp[0];
                    b_pos[1] = tmp[1];
                    simGetObjectOrientation(b_point, -1, tmp);
                    b_pos[2] = tmp[2];
                }
                
            }else if(loc == 2){
                //odometry
                
                float deltaS ;
                float deltaTheta;
                float deltalw;
                float deltarw;
                simFloat vd;
                simFloat wd;
                
                simGetJointPosition(motorLeft, &lwheel_pos);
                simGetJointPosition(motorRight, &rwheel_pos);
                //	std::cout <<"lwheel_pos     " <<lwheel_pos<< std::endl;
                //	std::cout <<"rwheel_pos     " <<rwheel_pos<< std::endl;
                deltarw=(rwheel_pos - past_rw_pos);
                deltalw=(lwheel_pos - past_lw_pos);
                deltaS = (wheel_radius / 2)*(deltarw + deltalw);
                deltaTheta = (wheel_radius  / axel_length )*(deltarw - deltalw);
                
                vd = deltaS / time_step;
                wd = deltaTheta / time_step;
                
                if(abs(wd) > 0.01){
                    //exact integration
                    currentConf[2] = past_pose[2] + deltaTheta;
                    currentConf[0] = past_pose[0] + (deltaS/deltaTheta) *(sin(currentConf[2]) - sin(past_pose[2]) );
                    currentConf[1] = past_pose[1] - (deltaS/deltaTheta) *(cos(currentConf[2]) - cos(past_pose[2]) );
                    
                }
                else{
                    //euler integration (used when wd is close to zero to avoid the division by deltaTheta (deltaTheta==0 if wd==0))
                    currentConf[0] = past_pose[0] + deltaS * cos(past_pose[2]);
                    currentConf[1] = past_pose[1] + deltaS * sin(past_pose[2]);
                    currentConf[2] = past_pose[2] + deltaTheta;
                }
                //if task is io static linearization estimate b_pos (estimate currentConf and compute b_pos from that)
                if (task==4){
                    b_pos[0] =  currentConf[0] + b_length*cos(currentConf[2]);
                    b_pos[1] =  currentConf[1] + b_length*sin(currentConf[2]);
                    b_pos[2] =  currentConf[2];
                }
                
                
            }
            
            for (int i = 0; i<3; i++){
                past_pose[i] = currentConf[i];
            }
            past_rw_pos = rwheel_pos;
            past_lw_pos = lwheel_pos;
            
            /*
             simFloat real_pose[3] = { 0, 0, 0 };
             simGetObjectPosition(ref_point, -1, real_pose);
             simFloat localization_error[3] = { 0, 0, 0 };
             for (int i = 0; i<3; i++){
             localization_error[i] = real_pose[i] - currentConf[i];
             }
             */
            
            
            
            // COMPUTE DESIRED CONFIGURATION
            if (task>= 1 && task <= 4){
                compute_desired_position(time);
            }
            
            
            // COMPUTE CONTROL VELOCITIES
            //this functions compute the control velocities w_r and w_l of the two wheels (global variables)
            switch(task){
                case 1:						//linear controller
                    linear_controller();
                    break;
                case 2:						// non linear controller
                    non_linear_controller();
                    break;
                case 3:						// IO dinamic linearization controller
                    dynamic_io_linearization(time);
                    break;
                case 4:						// IO static linearization controller
                    static_io_linearization();
                    break;
                case 5:						// cartesian regulation
                    cartesian_reg();
                    break;
                case 6:						// posture regulation
                    posture_reg();
                    break;
                default: ;
            }
            
            float saturation = 15;
            if(w_l > saturation){
                w_l = saturation;
            }
            if(w_l < -saturation){
                w_l = -saturation;
            }
            
            if(w_r > saturation){
                w_r = saturation;
            }
            if(w_r < -saturation){
                w_r = -saturation;
            }
            
            
            simSetJointTargetVelocity(motorLeft,w_l);
            simSetJointTargetVelocity(motorRight,w_r);
            
            
            if ( fmod(time,0.2) <= time_step ){
                tracking(ref_point);
            }
            
            
            // COMPUTE THE ACTUAL ERROR TO BE PLOTTED
            //compute the ACTUAL error:
            //in case of loc=1 it is (desiredConf[i]-currentConf[i])
            //in case of loc=2 currentConf is NOT the actual configuration, yet it has to be retrieved ideally
            float error[3]={0,0,0};
            if (loc == 1){
                for (int i = 0; i<3; i++){
                    error[i] = (desiredConf[i]-currentConf[i]);
                }
            }else{
                simFloat tmp[3] = {0,0,0};
                simGetObjectPosition(ref_point, -1, tmp);
                error[0] = desiredConf[0]-tmp[0];
                error[1] = desiredConf[1]-tmp[1];
                simGetObjectOrientation(ref_point, -1, tmp);
                error[2] = desiredConf[2]-tmp[2];
            }
            
            simFloat norm_err = sqrt(error[0]*error[0]+error[1]*error[1]);
            
            simFloat angle_err = bound_angle_error(error[2]);
            
            simFloat lyapunov = 0;
            simFloat lyapunov_dot = 0;
            
            
            // SEND ERROR TO THE ERROR GRAPH
            if (task != 2){
                simSetGraphUserData(error_graph_handler, "DistanceError", norm_err);
                simSetGraphUserData(error_graph_handler, "AngleError", angle_err);
            } else {
                lyapunov = k2/2 * (error[0]*error[0]+error[1]*error[1])+(error[2]*error[2])/2;
                lyapunov_dot = -k1*k2*error[0]*error[0] - k3 * error[2]*error[2];
                simSetGraphUserData(error_graph_handler, "DistanceError", norm_err);
                simSetGraphUserData(error_graph_handler, "AngleError", angle_err);
                simSetGraphUserData(error_graph_handler, "Lyapunov", lyapunov);
                
            }
            
            // SEND TRAJECTORIES TO THE TRAJECTORIES GRAPH
            PlotTrajectoryGraph();
            
            //PRINT DATA INTO THE FILE
            //norm error + tab + angular error + tab + v + tab + w  + tab + x + tab + y + tab + theta
            data_file << time << ", " << norm_err << ", " << angle_err << ", " << v << ", " << w << ", " << currentConf[0] << ", " << currentConf[1] << ", " <<  currentConf[2] <<"\n";
            
            // TERMINATION CONDITION FOR TRAJECTORY TRACKING
            if (time >= 1200 && task != 5 && task != 6){
                stopSim();
            }
            
        }
        
        
    }
    
    if (message==sim_message_eventcallback_mainscriptabouttobecalled)
    { // The main script is about to be run (only called while a simulation is running (and not paused!))
        
    }
    
    //**************************************************************************************************************//
    //******************************    ABOUT TO START    **********************************************************//
    //**************************************************************************************************************//
    
    if (message==sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start
        
        time_step = simGetSimulationTimeStep();
        
        // we have to take the robot's parameters
        motorLeft = simGetObjectHandle("Pioneer_p3dx_leftMotor");
        motorRight = simGetObjectHandle("Pioneer_p3dx_rightMotor");
        
        simFloat distance[3] = {0,0,0};
        
        simGetObjectPosition(motorLeft, motorRight, distance);
        
        axel_length = distance[2];
        wheel_radius = robot_height;
        
        robot = simGetObjectHandle("Pioneer_p3dx");
        ref_point = simGetObjectHandle("Ref_point");
        b_point = simGetObjectHandle("B_point");
        
        simFloat initConf[3] = {0,0,0};
        simFloat init[3] = {0,0,0};
        simGetObjectPosition(ref_point, -1, init);
        initConf[0] = init[0];
        initConf[1] = init[1];
        past_pose[0] = init[0];
        past_pose[1] = init[1];
        simGetObjectOrientation(ref_point, -1, init);
        initConf[2] = init[2];
        past_pose[2] = init[2];
        
        past_rw_pos = 0 ;
        past_lw_pos = 0 ;
        
        simSetJointPosition(motorLeft,past_lw_pos);
        simSetJointPosition(motorRight,past_rw_pos);
        
        for (int i = 0; i < 3; i++)
            currentConf[i] = initConf[i];
        // impose b_point position
        simFloat b_position[3] = {b_length,0,0};
        simSetObjectPosition(b_point, ref_point, b_position);
        
        if (task >= 1 && task <= 4){
            if (traj == 1)	{
                create_linear_traj();
                
                removable_handlers[0] = path_handle;
                removable_handlers[1] = path_dummy;
                
            }
            if (traj == 2)	{
                create_circular_traj();
                
                removable_handlers[0] = path_handle;
                removable_handlers[1] = path_dummy;
            }
            if (traj == 3)	{
                create_spline_traj();
                
            }
        }
        
        T = L/T;
        
        if (traj == 3) {
            spline_t1 = l1/L*T;
            spline_t2 = (l1+l2)/L*T;
            spline_t3 = (l1+l2+l3)/L*T;
            spline_t4 = T;
        }
        
        const simFloat color[12] = {1.0, 1.0, 0, 0,0,0, 1.0,1.0,0, 1.0,1.0,0};
        tracking_path = simCreatePath(-1, NULL, NULL, color);
        
        std::string track_name = "Tracking_Path";
        char tmpBuff[3];

        sprintf(tmpBuff,"%d", tracking_numb);
        track_name += tmpBuff;
        simSetObjectName(tracking_path, (simChar *)track_name.c_str());

        tracking_numb++;

        //retrieve graph handler
        error_graph_handler = simGetObjectHandle("ErrorGraph");
        trajectory_graph_handler = simGetObjectHandle("TrajectoryGraph");
        
        
        // Open data file
        data_file.open("data_file.txt");
        
        
        
    }
    
    if (message==sim_message_eventcallback_simulationended)
    { // Simulation just ended
        data_file.close();
    }
    /*
     if (message==sim_message_eventcallback_moduleopen)
     { // A script called simOpenModule (by default the main script). Is only called during simulation.
     if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
     {
     // we arrive here only at the beginning of a simulation
     }
     }
     
     if (message==sim_message_eventcallback_modulehandle)
     { // A script called simHandleModule (by default the main script). Is only called during simulation.
     if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
     {
     // we arrive here only while a simulation is running
     }
     }
     
     if (message==sim_message_eventcallback_moduleclose)
     { // A script called simCloseModule (by default the main script). Is only called during simulation.
     if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
     {
     // we arrive here only at the end of a simulation
     }
     }
     */
    if (message==sim_message_eventcallback_instanceswitch)
    { // We switched to a different scene. Such a switch can only happen while simulation is not running
        
    }
    
    if (message==sim_message_eventcallback_broadcast)
    { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)
        
    }
    
    if (message==sim_message_eventcallback_scenesave)
    { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)
        
    }
    
    // You can add many more messages to handle here
    
    if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
    { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag=false;
    }
    
    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}

// **************************************************************************************************************
// **********************************************CUSTOM FUNCTIONS***********************************************
// **************************************************************************************************************


void check_values(int init_val, int fin_val, double* param, double default_value){
    int i,j;
    simChar* label;
    char buff[256];
    for (i = init_val; i<=fin_val; i++){
        label = simGetUIButtonLabel(ui_handle, i);
        j = 0;
        
        while(label[j]!= NULL ){
            if (!isdigit(label[j]) && label[j] != '.'&& label[j] != '-') break;
            j++;
        }
        if (label[j] == NULL){
            param[i-init_val] = (double)atof(label);}
        else {
            param[i-init_val] = default_value;
            sprintf(buff, "%f", (float)default_value);
            simSetUIButtonLabel(ui_handle, i, buff, NULL);
        }
        
    }
	//buff[0] = NULL;
    
}

void create_linear_traj(){
    path_handle = simCreatePath(-1, NULL, NULL, NULL);
    start_point[0] = param[0];
    start_point[1] = param[1];
    start_point[2] = 0;
    end_point[0] = param[2];
    end_point[1] = param[3];
    end_point[2] = 0;
    float ptData[16] ={start_point[0],start_point[1],start_point[2],0,0,0,0,0,1,0,0};
    simInsertPathCtrlPoints(path_handle,0,0,1,ptData);
    float ptData1[16] = {end_point[0],end_point[1],end_point[2],0,0,0,0,0,1,0,0};
    simInsertPathCtrlPoints(path_handle,0,1,1,ptData1);
    
    L = sqrt((end_point[0]-start_point[0])*(end_point[0]-start_point[0])+(end_point[1]-start_point[1])*(end_point[1]-start_point[1]));
    
    simFloat dummy_attr[12] = {0,1,0,0,0,0,0,0,0,0,0,0};
    path_dummy = simCreateDummy((simFloat)0.02,dummy_attr);
    simSetObjectName(path_dummy, "Path Follower Dummy");
    simSetObjectPosition(path_dummy, -1, start_point);
}

void create_circular_traj(){
    
    radius = param[2];
    xc = param[0];
    yc = param[1];
    path_handle = simCreatePath(-1 , NULL, NULL, NULL);
    const int n_points = 100;
    float ptData[n_points*11];
    for(int i = 0; i< n_points; i++){
        
        ptData[11*i+0] = radius*cos(2*pi/n_points*i)+xc;
        ptData[11*i+1] = radius*sin(2*pi/n_points*i)+yc;
        ptData[11*i+2] = 0;
        ptData[11*i+3] = 0;
        ptData[11*i+4] = 0;
        ptData[11*i+5] = atan2(sin(2*pi/n_points*i), cos(2*pi/n_points*i));
        ptData[11*i+6] = 0;
        ptData[11*i+7] = (float)i/n_points;
        ptData[11*i+8] = 1;
        ptData[11*i+9] = 0;
        ptData[11*i+10] = 0;
        
    }
    simInsertPathCtrlPoints(path_handle, 1, 0, n_points, ptData);
    
    L = 2 * pi * radius;
    
    start_point[0]=ptData[0];
    start_point[1]=ptData[1];
    start_point[2]=ptData[2];
    end_point[0]=ptData[0];
    end_point[1]=ptData[1];
    end_point[2]=ptData[2];
    simFloat dummy_attr[12] = {0,1,0,0,0,0,0,0,0,0,0,0};
    path_dummy = simCreateDummy((simFloat)0.02,dummy_attr);
    simSetObjectName(path_dummy, "Path Follower Dummy");
    simSetObjectPosition(path_dummy, -1, start_point);
}

void create_spline_traj(){
    
    simFloat dummy_attr1[12] = {0,1,0,0,0,0,0,0,0,0,0,0};
    simInt spline_dummy[5];
    simFloat p[3];
    
    for (int i = 0; i < 5; i++){
        
        p[0] = param[0+i*2];
        p[1] = param[1+i*2];
        p[2] = 0;
        spline_dummy[i] = simCreateDummy((simFloat)0.05,dummy_attr1);
        
        // dummy can be removed in the next running
        removable_handlers[i] = spline_dummy[i];
        
        simSetObjectPosition(spline_dummy[i], -1, p);
        
    }
    
    
    //creation of the system matrices
    Eigen::MatrixXf A = create_matrix();
    
    Eigen::VectorXf bx, by;
    
    bx = Eigen::VectorXf::Zero(16);
    by = Eigen::VectorXf::Zero(16);
    bx[0] = param[0];
    bx[1] = bx[2] = param[2];
    bx[3] = bx[4] = param[4];
    bx[5] = bx[6] = param[6];
    bx[7] = param[8];
    by[0] = param[1];
    by[1] = by[2] = param[3];
    by[3] = by[4] = param[5];
    by[5] = by[6] = param[7];
    by[7] = param[9];
    
    A.lu().solve(bx, &vx);
    A.lu().solve(by, &vy);
    
    double t;
    
    simInt path_handle[4];
    const int n_points = 30;
    double n_points2 = 30;
    float ptData[n_points*11];
    int base_index = 0;
    simInt path_prop[3] = {1, sim_distcalcmethod_dl, 0};
    
    for(int j = 0; j< 4; j++){
        path_handle[j] = simCreatePath(-1, path_prop, NULL, NULL);
        for(int i = 0; i < n_points; i++){
            t = (double)(j+(double)(i/n_points2));
            base_index = 11*i;
            ptData[base_index+0] = vx[j*4]*pow(t,3)+vx[j*4+1]*pow(t,2)+vx[j*4+2]*t+vx[j*4+3];
            ptData[base_index+1] = vy[j*4]*pow(t,3)+vy[j*4+1]*pow(t,2)+vy[j*4+2]*t+vy[j*4+3];
            ptData[base_index+2] = 0;
            ptData[base_index+3] = 0;
            ptData[base_index+4] = 0;
            ptData[base_index+5] = atan2(3*vy[j*4]*pow(t,2)+2*vy[j*4+1]*t+vy[j*4+2] ,3*vx[j*4]*pow(t,2)+2*vx[j*4+1]*t+vx[j*4+2]);
            ptData[base_index+6] = 0;
            ptData[base_index+7] = (float)i/n_points2;
            ptData[base_index+8] = 3;
            ptData[base_index+9] = 0;
            ptData[base_index+10] = 0;
        }
        simInsertPathCtrlPoints(path_handle[j], 0, 0, n_points, ptData);
        
        //various splines can be removed in the next running
        removable_handlers[j+5] = path_handle[j];
        
        
    }
    
    simGetPathLength(path_handle[0], &l1);
    simGetPathLength(path_handle[1], &l2);
    simGetPathLength(path_handle[2], &l3);
    simGetPathLength(path_handle[3], &l4);
    
    L = l1+l2+l3+l4;
    
    start_point[0]=ptData[0];
    start_point[1]=ptData[1];
    start_point[2]=ptData[2];
    
    simFloat dummy_attr[12] = {0,1,0,0,0,0,0,0,0,0,0,0};
    path_dummy = simCreateDummy((simFloat)0.02,dummy_attr);
    
    removable_handlers[9] = path_dummy;
    
    simSetObjectName(path_dummy, "Path Follower Dummy");
    simSetObjectPosition(path_dummy, -1, start_point);
}




Eigen::MatrixXf create_matrix(){
    double ss1,ss2,ss3,ss4;
    double sf1,sf2,sf3,sf4;
    double tv[8] = {0, 1, 1, 2, 2, 3, 3, 4};
    ss1 = tv[0]; ss2 = tv[2]; ss3 = tv[4]; ss4 = tv[6];
    sf1 = tv[1]; sf2 = tv[3]; sf3 = tv[5]; sf4 = tv[7];
    
    int n_var = 16;
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(n_var,n_var);
    
    A(0,0) = pow(ss1,3);
    A(0,1) = pow(ss1, 2);
    A(0,2) = ss1;
    A(0,3) = 1;
    A(1,0) = pow(sf1,3);
    A(1,1) = pow(sf1,2);
    A(1,2) = sf1;
    A(1,3) = 1;
    
    A(2,4) = pow(ss2,3);
    A(2,5) = pow(ss2,2);
    A(2,6) = ss2;
    A(2,7) = 1;
    A(3,4) = pow(sf2,3);
    A(3,5) = pow(sf2,2);
    A(3,6) = sf2;
    A(3,7) = 1;
    
    A(4,8)	=	pow(ss3,3);
    A(4,9)	=	pow(ss3,2);
    A(4,10) =	ss3;
    A(4,11) =	1;
    A(5,8)	=	pow(sf3,3);
    A(5,9)	=	pow(sf3,2);
    A(5,10) =	sf3;
    A(5,11) =	1;
    
    A(6,12) =	pow(ss4,3);
    A(6,13) =	pow(ss4,2);
    A(6,14) =	ss4;
    A(6,15) =	1;
    A(7,12) =	pow(sf4,3);
    A(7,13) =	pow(sf4,2);
    A(7,14) =	sf4;
    A(7,15) =	1;
    
    
    A(8,0) = 3*pow(ss1, 2);
    A(8,1) = 2*ss1;
    A(8,2) = 1;
    
    A(9,0) = 3*pow(sf1, 2);
    A(9,1) = 2*sf1;
    A(9,2) = 1;
    A(9,4) = -3*pow(ss2, 2);
    A(9,5) = -2*ss2;
    A(9,6) = -1;
    
    A(10,4)	= 3*pow(sf2, 2);
    A(10,5)	= 2*sf2;
    A(10,6)	= 1;
    A(10,8)	= -3*pow(ss3, 2);
    A(10,9)	= -2*ss3;
    A(10,10)= -1;
    
    A(11,8)	 = 3*pow(sf3, 2);
    A(11,9)	 = 2*sf3;
    A(11,10) = 1;
    A(11,12) = -3*pow(ss4, 2);
    A(11,13) = -2*ss4;
    A(11,14) = -1;
    
    A(12,12) = 3*pow(sf4, 2);
    A(12,13) = 2*sf4;
    A(12,14) = 1;
    
    A(13,0) = 6*sf1;
    A(13,1) = 2;
    A(13,4) = -6*ss2;
    A(13,5) = -2;
    
    A(14,4) = 6*sf2;
    A(14,5) = 2;
    A(14,8) = -6*ss3;
    A(14,9) = -2;
    
    A(15,8)  = 6*sf3;
    A(15,9)  = 2;
    A(15,12) = -6*ss4;
    A(15,13) = -2;
    return A;
}




double s = 0.0;
double c;
void compute_desired_position(double t){
    
    simFloat desiredPosition[3];
    
    switch(traj){
        case 1:
            s = t/T;
            if(t>T)	return;
            desiredConf[0] = start_point[0]+s*(end_point[0]-start_point[0]);
            desiredConf[1] = start_point[1]+s*(end_point[1]-start_point[1]);
            desiredConf[2] = atan2(end_point[1]-start_point[1],end_point[0]-start_point[0]);
            desiredPosition[0] = desiredConf[0];
            desiredPosition[1] = desiredConf[1];
            desiredPosition[2] = 0;
            break;
        case 2:
            s = t/T;
            radius = param[2];
            xc = param[0];
            yc = param[1];
            desiredConf[0] = radius*cos(s*2*pi)+xc;
            desiredConf[1] = radius*sin(s*2*pi)+yc;
            desiredConf[2] = atan2(cos(s*2*pi), -sin(s*2*pi));
            
            desiredPosition[0] = desiredConf[0];
            desiredPosition[1] = desiredConf[1];
            desiredPosition[2] = 0;
            break;
        case 3:							//splines
            double s_in;
            int j = 3;
            
            if(t>= 0 && t< spline_t1){
                spline_tin = 0;
                spline_tfin = spline_t1;
                spline_lin = 0;
                spline_lfin = l1;
                s_in = 0;
                j = 0;
            } else if (t>= spline_t1 && t< spline_t2){
                spline_tin = spline_t1;
                spline_tfin = spline_t2;
                spline_lin = l1;
                spline_lfin = l1+l2;
                s_in = 1;
                j = 1;
            } else if (t>= spline_t2 && t< spline_t3){
                spline_tin = spline_t2;
                spline_tfin = spline_t3;
                spline_lin = l1+l2;
                spline_lfin = l1+l2+l3;
                s_in = 2;
                j = 2;
            } else if (t>= spline_t3 && t<= spline_t4){
                spline_tin = spline_t3;
                spline_tfin = spline_t4;
                spline_lin = l1+l2+l3;
                spline_lfin = L;
                s_in = 3;
                j = 3;
            } else return;
            double arg = s_in+ (((t-spline_tin)/(spline_tfin-spline_tin))*(spline_lfin-spline_lin))/(spline_lfin-spline_lin);
            desiredConf[0] = vx[j*4]*pow(arg,3)+vx[j*4+1]*pow(arg,2)+vx[j*4+2]*arg+vx[j*4+3];
            desiredConf[1] = vy[j*4]*pow(arg,3)+vy[j*4+1]*pow(arg,2)+vy[j*4+2]*arg+vy[j*4+3];
            desiredConf[2] = atan2(3*vy[j*4]*pow(arg,2)+2*vy[j*4+1]*arg+vy[j*4+2] ,3*vx[j*4]*pow(arg,2)+2*vx[j*4+1]*arg+vx[j*4+2]);
            desiredPosition[0] = desiredConf[0];
            desiredPosition[1] = desiredConf[1];
            desiredPosition[2] = 0;
            break;
    }
    simSetObjectPosition(path_dummy, -1, desiredPosition);
    simFloat desiredOrientation[3] = {0,0,desiredConf[2]};
    simSetObjectOrientation(path_dummy, -1, desiredOrientation);
}


double  bound_angle_error(double e){
    while(e < -pi){
        e = e+ 2*pi;
    }
    while(e > pi){
        e = e - 2*pi;
    }
    return e;
}

void linear_controller(){
    // compute conf error
    float error[3]={0,0,0};
    float error_tilde[3]={0,0,0};
    for (int i = 0; i<3; i++){
        error_tilde[i] = (desiredConf[i]-currentConf[i]);
        
    }
    // rotate error
    error[0] = error_tilde[0]*cos(currentConf[2])+error_tilde[1]*sin(currentConf[2]);
    error[1] = error_tilde[1]*cos(currentConf[2])-error_tilde[0]*sin(currentConf[2]);
    error[2] = error_tilde[2];
    
    error[2] = bound_angle_error(error[2]);
        
    // compute desired velocities
    simFloat vd[3]={0,0,0};
    simFloat wd[3]={0,0,0};
    
    simGetObjectVelocity(path_dummy, vd, wd);
    
    simFloat norm_vd = sqrt(vd[0]*vd[0]+vd[1]*vd[1]+vd[2]*vd[2]);
    simFloat norm_wd = sqrt(wd[0]*wd[0]+wd[1]*wd[1]+wd[2]*wd[2]);
    
    
    
    // compute gains 
    
    if (gain_mod == 1){
        k1 = k_gains[0];
        k2 = k_gains[1];
        k3 = k_gains[2];
    }else{
        double chi = k_gains[0];
        double a = k_gains[1];
        k1 = 2*chi*a;
        k3 = k1;
        if (norm_vd != 0)
            k2 = ((a*a)-(norm_wd*norm_wd))/(norm_vd);
        else
            k2 = ((a*a)-(norm_wd*norm_wd))*10000000;
    }
    
    // compute control velocities
    
    simFloat u1 = - k1 * error[0];
    simFloat u2 = - k2 * error[1] - k3 * error[2];
    
    v = norm_vd*cos(error[2])-u1;
    w = norm_wd-u2;
    
    
    w_l = (1/(2*wheel_radius))*(2*v-axel_length*w);
    w_r = (1/(2*wheel_radius))*(2*v+axel_length*w);
    
}

void non_linear_controller(){
    
    // compute conf error
    float error[3]={0,0,0};
    float error_tilde[3]={0,0,0};
    for (int i = 0; i<3; i++){
        error_tilde[i] = (desiredConf[i]-currentConf[i]);
        
        
    }
    // rotate error
    error[0] = error_tilde[0]*cos(currentConf[2])+error_tilde[1]*sin(currentConf[2]);
    error[1] = error_tilde[1]*cos(currentConf[2])-error_tilde[0]*sin(currentConf[2]);
    error[2] = error_tilde[2];
    
    error[2] = bound_angle_error(error[2]);
    
    simFloat norm_err = sqrt(error[0]*error[0]+error[1]*error[1]+error[2]*error[2]);
    
    // compute desired velocities
    simFloat vd[3]={0,0,0};
    simFloat wd[3]={0,0,0};
    
    simGetObjectVelocity(path_dummy, vd, wd);
    
    simFloat norm_vd = sqrt(vd[0]*vd[0]+vd[1]*vd[1]+vd[2]*vd[2]);
    simFloat norm_wd = sqrt(wd[0]*wd[0]+wd[1]*wd[1]+wd[2]*wd[2]);
    
    
    k1 = k_gains[0];
    k2 = k_gains[1];
    k3 = k_gains[2];
    
    simFloat u1 = -k1*error[0];
    simFloat u2 = 0;
    if (error[2]!=0){
        u2 = -k2*norm_vd*sin(error[2])/error[2]*error[1]-k3*error[2];
    } else {
        u2 = -k2*norm_vd*sin(error[2])*10000000*error[1]-k3*error[2];
    }
    v = norm_vd*cos(error[2])-u1;
    w = norm_wd-u2;
    
    
    w_l = (1/(2*wheel_radius))*(2*v-axel_length*w);
    w_r = (1/(2*wheel_radius))*(2*v+axel_length*w);
}

void static_io_linearization(){
    
    simFloat vd[3]={0,0,0};
    simFloat wd[3]={0,0,0};
    
    simGetObjectVelocity(path_dummy, vd, wd);
    
    k1 = k_gains[0];
    k2 = k_gains[1];
    
    double u1 = vd[0]+k1*(desiredConf[0]-b_pos[0]);
    double u2 = vd[1]+k2*(desiredConf[1]-b_pos[1]);
    
    v = cos(currentConf[2])*u1+sin(currentConf[2])*u2;
    w = -sin(currentConf[2])/b_length*u1+cos(currentConf[2])/b_length*u2;
    
    w_l = (1/(2*wheel_radius))*(2*v-axel_length*w);
    w_r = (1/(2*wheel_radius))*(2*v+axel_length*w);
}

void dynamic_io_linearization(double t){
    
    simFloat vd[3]={0,0,0};
    simFloat wd[3]={0,0,0};
    simGetObjectVelocity(path_dummy, vd, wd);
    
    simFloat ad[3]={0,0,0};
    switch(traj){
        case 1:
            // nothing 'cause acc == {0,0,0} in linear trajectory
            break;
        case 2:
            // for constant linear velocity on circular path
            ad[0] = -radius*4*pi*pi/(T*T)*cos(2*pi*t/T);
            ad[1] = -radius*4*pi*pi/(T*T)*sin(2*pi*t/T);
            break;
        case 3:
            break;
        default:;
    }
    
    // velocity of the b point
    simFloat vb[3]={0,0,0};
    simFloat wb[3]={0,0,0};
    simGetObjectVelocity(ref_point, vb, wb);
    
    k1 = k_gains[0];
    k2 = k_gains[1];
    k3 = k_gains[2];
    k4 = k_gains[3];
    
    double u1 = ad[0]+k1*(desiredConf[0]-currentConf[0])+k2*(vd[0]-vb[0]);
    double u2 = ad[1]+k3*(desiredConf[1]-currentConf[1])+k4*(vd[1]-vb[1]);
    
    simFloat a = u1*cos(currentConf[2])+sin(currentConf[2])*u2;
    if (t >= 0 && t<= simGetSimulationTimeStep()){
        v = sqrt(vd[0]*vd[0]+vd[1]*vd[1]+vd[2]*vd[2]);
        
    }else{
        
        v_old = sqrt(vb[0]*vb[0]+vb[1]*vb[1]+vb[2]*vb[2]);
        v = v_old + a*(simGetSimulationTimeStep());
    }
    //reinitialization: if chi goes down a certain treshold it's reinitialized
    if (v <= 0.5) {
        v = sqrt(vd[0]*vd[0]+vd[1]*vd[1]+vd[2]*vd[2]);
    }
    simFloat w = (-sin(currentConf[2])*u1+cos(currentConf[2])*u2)/v;
    
    w_l = (1/(2*wheel_radius))*(2*v-axel_length*w);
    w_r = (1/(2*wheel_radius))*(2*v+axel_length*w);
    
}

void cartesian_reg()
{
    k1 = k_gains[0];
    k2 = k_gains[1];
    
    v = -k1*(currentConf[0] * cos(currentConf[2]) + currentConf[1] * sin(currentConf[2]));
    w = k2*bound_angle_error(atan2(currentConf[1], currentConf[0])  - currentConf[2] + pi);
    
    w_l = (1 / (2 * wheel_radius))*(2 * v - axel_length*w);
    w_r = (1 / (2 * wheel_radius))*(2 * v + axel_length*w);
    
    double norm = sqrt(currentConf[0] * currentConf[0] + currentConf[1] * currentConf[1]);    
    
    if (norm < 0.03) { 
        stopSim();
    }
}


void posture_reg()
{
    rho = sqrt(currentConf[0] * currentConf[0] + currentConf[1] * currentConf[1]);
    gam = atan2(currentConf[1], currentConf[0]) - currentConf[2] + pi;
    delta = gam + currentConf[2];
    
    //error bounding
    gam = bound_angle_error(gam);
    delta = bound_angle_error(delta);
    
    k1 = k_gains[0];
    k2 = k_gains[1];
    k3 = k_gains[2];
    
    v = k1*rho*cos(gam);
    w = k2*gam + (k1 / gam)*(sin(gam)*cos(gam)*(gam + k3*delta));
    
    w_l = (1 / (2 * wheel_radius))*(2 * v - axel_length*w);
    w_r = (1 / (2 * wheel_radius))*(2 * v + axel_length*w);
    double norm = sqrt(currentConf[0] * currentConf[0] + currentConf[1] * currentConf[1]);
    
    if (norm < 0.03 && currentConf[2] < 0.087 && currentConf[2] >- 0.087) { 
        stopSim();
    }
}





void stopSim(){
    simGetObjectPosition(robot, -1, finalPos);
    simGetObjectOrientation(robot, -1, finalOrient);
    simStopSimulation(); 
}


void tracking(simInt obj){
    
    simFloat pos[3] = {0,0,0};
    simFloat orient[3] = {0,0,0};
    
    simGetObjectPosition(obj, -1, pos);
    simGetObjectOrientation(obj, -1, orient);
    
    //***************************************************************************************************** in case of io linear (mod==4) we have to take pos and orient from the b_point
    
    float ptData[11];
    
    ptData[0] = pos[0];
    ptData[1] = pos[1];
    ptData[2] = 0;
    ptData[3] = 0;
    ptData[4] = 0;
    ptData[5] = orient[2];
    ptData[6] = 0;
    ptData[7] = 1;
    ptData[8] = 3;
    ptData[9] = 0;
    ptData[10] = 0;
    
    simInsertPathCtrlPoints(tracking_path, 0, tracking_point_number, 1, ptData);
    
    tracking_point_number += 1;
    
}
void PlotTrajectoryGraph(){
    
    simSetGraphUserData(trajectory_graph_handler, "xDesired", desiredConf[0]);
    simSetGraphUserData(trajectory_graph_handler, "yDesired",desiredConf[1]);
    
    
    if(loc==1){
        simSetGraphUserData(trajectory_graph_handler, "xCurrent", currentConf[0]);
        simSetGraphUserData(trajectory_graph_handler, "yCurrent", currentConf[1]);
    }
    else{
        
        simFloat tmp[3] = {0,0,0};
        simGetObjectPosition(ref_point, -1, tmp);
        simSetGraphUserData(trajectory_graph_handler, "xCurrent",tmp[0] ); //actual ref_point position
        simSetGraphUserData(trajectory_graph_handler, "yCurrent",tmp[1] );
        simSetGraphUserData(trajectory_graph_handler, "xOdometric", currentConf[0]); //thinked ref_point position
        simSetGraphUserData(trajectory_graph_handler, "yOdometric", currentConf[1]);
    }
    
    
    
}
void CreateObstacles() {
    
    for(int i=-3;i<3;i++){
        for(int j=-3;j<3;j++){
            simInt self=simGetObjectAssociatedWithScript(sim_handle_self);
            const simFloat sizetbox[3]={0.008, 0.008, 0.5};
            tbox = simCreatePureShape(0, 8, sizetbox,2, NULL);	
            const simFloat positiontbox[3]={i,j,5};
            simSetObjectPosition(tbox, self,positiontbox);
            simSetObjectName(tbox, "box1");
        } 
    } 
}
