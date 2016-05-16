#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <boost/asio.hpp>
#include <iostream>

int main(int argc,char** argv){

  double matrix28[9] = {9.910925439, 1.231185466, 0.970859841, -1.973447799, 14.66994936, -0.921719128, 0.630990042, -0.77121074, 55.61331789};
  double matrix33[9] = {11.38555711, 1.075823951, 0.341612057, -1.334627966, 13.00710247, -2.036977959, 0.909452516, -2.554647371, 51.72511683};
  double matrix8[9] = {11.15747603, 1.216047695, 2.318814325, -1.728443747, 12.10689728, 0.095171634, 0.188682325, -1.611091136, 51.28225561};
  double* matrixes[3] = {matrix28, matrix8, matrix33};

  std::string rline;
  std::size_t length;

  boost::asio::streambuf rbuf;
  const char *port_name = "/dev/touchsensor";
  boost::asio::io_service io;
  boost::asio::serial_port port( io, port_name );

  ros::init(argc,argv,"touch_sensor");
  ros::NodeHandle nh;

  ros::Publisher force1_pub = nh.advertise<geometry_msgs::Vector3>("touchence/force01", 100);
  ros::Publisher force2_pub = nh.advertise<geometry_msgs::Vector3>("touchence/force02", 100);
  ros::Publisher force3_pub = nh.advertise<geometry_msgs::Vector3>("touchence/force03", 100);
  ros::Publisher force_pubs[3] = {force1_pub, force2_pub, force3_pub};

  ros::Publisher temp1_pub = nh.advertise<std_msgs::Float32>("touchence/temp01", 100);
  ros::Publisher temp2_pub = nh.advertise<std_msgs::Float32>("touchence/temp02", 100);
  ros::Publisher temp3_pub = nh.advertise<std_msgs::Float32>("touchence/temp03", 100);
  ros::Publisher temp_pubs[3] = {temp1_pub, temp2_pub, temp3_pub};

  port.set_option(boost::asio::serial_port_base::baud_rate(230400));
  port.set_option(boost::asio::serial_port_base::character_size(8));
  port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

  //port.write_some(boost::asio::buffer(send_bin));
  char* e;
  long long_num[3],long_temp;
  double calib_tmp[3][3],calib_offset[3][3],calib_temp_tmp[3],calib_temp_offset[3];;
  int calib_count = 0;
  double temp_k[3] = {-12,-5,-9};

  std::cout << "calibration ready" << std::endl;
  std::cout << "dont touch the force sensors" << std::endl;
  sleep(1);
  std::cout << "calibration start" << std::endl;
  for(int i=0;i<3;i++){
    calib_tmp[i][0] = 0;
    calib_tmp[i][1] = 0;
    calib_tmp[i][2] = 0;
    calib_temp_tmp[i] = 0;
  }

  for(int i=0;i<50;i++){
    std::cout << "call" << std::endl;
    port.write_some(boost::asio::buffer("020201"));
    length = boost::asio::read_until(port,rbuf,'\n');
    std::cout << boost::asio::buffer_cast<const char*>(rbuf.data()) << std::endl;

    
    std::cout << length << std::endl;
    std::string tmp = boost::asio::buffer_cast<const char*>(rbuf.data());
    std::cout << tmp << std::endl;
    if(length == 68){
      for(int i=0;i<3;i++){
	  long_num[0] = std::strtol(tmp.substr(2 + i*16,4).c_str(),&e,16);
	  long_num[1] = std::strtol(tmp.substr(6 + i*16,4).c_str(),&e,16);
	  long_num[2] = std::strtol(tmp.substr(10 + i*16,4).c_str(),&e,16);
	  long_temp = std::strtol(tmp.substr(14 + i*16,4).c_str(),&e,16);

	  calib_tmp[i][0] += matrixes[i][0]*long_num[0] + matrixes[i][1]*long_num[1] + matrixes[i][2]*long_num[2];
	  calib_tmp[i][1] += matrixes[i][3]*long_num[0] + matrixes[i][4]*long_num[1] + matrixes[i][5]*long_num[2];
	  calib_tmp[i][2] += matrixes[i][6]*long_num[0] + matrixes[i][7]*long_num[1] + matrixes[i][8]*long_num[2];
	  calib_temp_tmp[i] += long_temp;
      }
      calib_count++;
    }
    rbuf.consume(length);
    usleep(100000);
  }

  if(calib_count == 0){
    ROS_ERROR("cant read force");
    return -1;
  }


  for(int i=0; i<3; i++){
    calib_offset[i][0] = calib_tmp[i][0]/calib_count;
    calib_offset[i][1] = calib_tmp[i][1]/calib_count;
    calib_offset[i][2] = calib_tmp[i][2]/calib_count;
    calib_temp_offset[i] = calib_temp_tmp[i]/calib_count;
  }
  std::cout << "calibration finished" << std::endl;

  ros::Rate loop_rate(10);
  while(ros::ok()){

    port.write_some(boost::asio::buffer("020201"));
    length = boost::asio::read_until(port,rbuf,'\n');
    std::cout << boost::asio::buffer_cast<const char*>(rbuf.data()) << std::endl;

    
    std::cout << length << std::endl;
    std::string tmp = boost::asio::buffer_cast<const char*>(rbuf.data());
    std::cout << tmp << std::endl;

    if(length == 68){
      for(int i=0;i<3;i++){
	  geometry_msgs::Vector3 tmp_msg;
	  std_msgs::Float32 temp_msg;

	  long_num[0] = std::strtol(tmp.substr(2 + i*16,4).c_str(),&e,16);
	  long_num[1] = std::strtol(tmp.substr(6 + i*16,4).c_str(),&e,16);
	  long_num[2] = std::strtol(tmp.substr(10 + i*16,4).c_str(),&e,16);
	  long_temp = std::strtol(tmp.substr(14 + i*16,4).c_str(),&e,16);

	  tmp_msg.x = matrixes[i][0]*long_num[0] + matrixes[i][1]*long_num[1] + matrixes[i][2]*long_num[2] - calib_offset[i][0];
	  tmp_msg.y = matrixes[i][3]*long_num[0] + matrixes[i][4]*long_num[1] + matrixes[i][5]*long_num[2] - calib_offset[i][1];
	  tmp_msg.z = matrixes[i][6]*long_num[0] + matrixes[i][7]*long_num[1] + matrixes[i][8]*long_num[2] - calib_offset[i][2] - (long_temp - calib_temp_offset[i]) * temp_k[i];
	  temp_msg.data = long_temp - calib_temp_offset[i];


	  // 10bit3.3V maybe
	  tmp_msg.x /= 310;
	  tmp_msg.y /= 310;
	  tmp_msg.z /= 310;
	  temp_msg.data /= 310;

	  force_pubs[i].publish(tmp_msg);
	  temp_pubs[i].publish(temp_msg);
      }
      std::cout << std::endl;
    }
    rbuf.consume(length);
    loop_rate.sleep();
    }
  return 0;
}

