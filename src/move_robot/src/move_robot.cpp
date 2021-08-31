#include "move_robot.h"
#include "AllDir.h"
#include "OneWheel_vw.h"
#include "test_package.h"
//#include "DiffV.h"
//#include "boling.h"
//#include "Diff_vlvr.h"
//#include "Diff_vw.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::Time::init();
    ros::NodeHandle now;
    ros::Subscriber TriggerSubscriber_=now.subscribe("Trigger", 5, TriggerCallback);
    int CarKind= -1;;		//車種()


    if(argc < 3){

        ROS_INFO("usage: [<Device name>] [<Baud rate>]");

        return 0;

    }



    baudrate = std::atoi(argv[2]);
    argv_buf = argv;

    LoadTitlePath();
    CarParameterPATH = TitlePath + CarParameterPATH_Local;

    int check = LoadCarKind(CarParameterPATH,CarKind);
    while(!check && ros::ok())
    {
        if(ReloadCarKind){
             LoadCarKind(CarParameterPATH,CarKind);
             break;
        }
        ros::spinOnce();
    }

    kind_search(CarKind, argv_buf, baudrate);


    ros::spin();

    return 0;


}
void TriggerCallback(const std_msgs::Int8& msg)
{
    int Trigger = msg.data;
    int CarKind = -1;

    switch(Trigger){
            case ReloadCarParameter:
                 ReloadCarKind = true;
                 ROS_INFO("ReloadCarParameter!!");
                 LoadCarKind(CarParameterPATH,CarKind);
                 kind_search(CarKind, argv_buf, baudrate);
            break;

            default:
            break;
    }
}
bool LoadCarKind(std::string file_buf,int &returnbuf)
{

    std::fstream fin;

    char *file = const_cast<char *>(file_buf.c_str());
    fin.open(file, std::fstream::in);
    if(!fin.is_open())
    {
        ROS_INFO("Error: CarParameter is not opened!!");
        return false;
    }
    else{
        ROS_INFO("the file is opened!!");

        std::string s_CarName;
        std::getline(fin, s_CarName);
        std::getline(fin, s_CarName);
        //CarName = s_CarName;

        std::string s_Carnumber;
        std::getline(fin, s_Carnumber);
        std::getline(fin, s_Carnumber);
        //Carnumber = std::atoi(s_Carnumber.c_str());

        std::string s_CarKind;
        std::getline(fin, s_CarKind);
        std::getline(fin, s_CarKind);
        returnbuf = std::atoi(s_CarKind.c_str());
    }

    fin.close();
    return true ;


}
void LoadTitlePath()
{

    std::string NOWPath = ros::package::getPath("AnhungControl");

	std::string PATH_par;
	std::string recv_pkg[100];

	int count=0;
    std::stringstream cut(NOWPath);
    while(getline(cut,PATH_par,'/'))
    {
        recv_pkg[count]=PATH_par;
        count++;
    }

	TitlePath = "/" + recv_pkg[1] + "/" + recv_pkg[2] + "/" + recv_pkg[3] + "/" + recv_pkg[4];
	std::cout<<"TitlePath  " <<TitlePath <<std::endl;

}
void kind_search(int CarKind ,char **argv ,int baudrate)
{
    switch(CarKind){
            case 0:
                 std::cout<<"CarKind = four  "<<std::endl;
                 alldir *four_wheel;
                 four_wheel = new alldir(argv[1], baudrate);
            break;
            case 1:
                 std::cout<<"CarKind = one  "<<std::endl;
                 onewheel_vw *one_wheel;
                 one_wheel = new onewheel_vw(argv[1], baudrate);
            break;
            case 2:
                 std::cout<<"CarKind = test  "<<std::endl;
                 test_package *test_pack;
                 test_pack = new test_package(argv[1], baudrate);
            break;
            default:
            break;
    }
}
