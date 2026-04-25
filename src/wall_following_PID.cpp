#include "navigation_node.hpp"
#include <vector>
#include <numeric>
#include <algorithm>
#include "std_msgs/msg/float64.hpp"
/**
 * @brief Exemplo simples de navegação reativa em C++.
 *
 * Esta classe implementa um algoritmo básico de navegação: vira em direção ao objetivo,
 * anda em linha reta se alinhado, e para se houver obstáculo à frente.
 *
 * Thresholds configuráveis:
 * - DIST_THRESHOLD: Distância para considerar objetivo atingido (0.1 m).
 * - ANGLE_THRESHOLD: Ângulo para considerar alinhado (0.1 rad ~ 5.7°).
 * - OBSTACLE_THRESHOLD: Distância para detectar obstáculo (0.5 m).
 */


class ControlExample : public NavigationNode
{
public:
  ControlExample() : NavigationNode("PID_cpp") {}

  std_msgs::msg::Float64 err, prev_err, dist, sonar1_msg, sonar2_msg;
  double p = 1;
  double i = 0.1;
  double d = 1;
  std::vector<double> err_reading = {0,0,0,0,0,0,0,0,0,0};
  std::vector<double> sonar1 = {0,0,0,0,0,0,0,0,0};
  std::vector<double> sonar2 = {0,0,0,0,0,0,0,0,0};
  std::vector<double> sonar1_filtered = {0,0,0,0,0,0,0,0,0};
  std::vector<double> sonar2_filtered = {0,0,0,0,0,0,0,0,0};
  const double OBSTACLE_DETECTION = 0.8; // metros
  const double OBSTACLE_THRESHOLD = 1;
  int iterator = 0;
  double linear = 0.2;
  double angular;
  double angle_err;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pub_ = 
    this->create_publisher<std_msgs::msg::Float64>("err", 0);
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr prev_err_pub_ = 
    this->create_publisher<std_msgs::msg::Float64>("prev_err", 0);
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sonar1_pub_ = 
    this->create_publisher<std_msgs::msg::Float64>("sonar1", 0);
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sonar2_pub_ = 
    this->create_publisher<std_msgs::msg::Float64>("sonar2", 0);
  int state = 0;
  double alinhamento; 
  bool direita = false;
  int indice_sonar1;
  int indice_sonar2;
  int indice_turn;

private:

  void follow_line(){
    double dist_to_goal = distance_to_goal();

    // Thresholds
    const double DIST_THRESHOLD = 0.1;     // metros
    const double ANGLE_THRESHOLD = 0.1;    // radianos (~5.7 graus)

    if (dist_to_goal < DIST_THRESHOLD) {
      // Objetivo alcançado
      clear_goal();
      stop();
      RCLCPP_INFO(this->get_logger(), "Objetivo alcançado!");
      return;
    }


    if (std::abs(angle_err) > ANGLE_THRESHOLD) {
      // Virar em direção ao objetivo
      angular = (angle_err > 0) ? 0.5 : -0.5; // velocidade angular
      publish_velocity(0.0, angular);
    } else {
      // Andar em linha reta
      publish_velocity(linear, 0.0); // velocidade linear
    }
  }
  
  void pre_wall_following(){
    if(direita) publish_velocity(0, 1);
    else publish_velocity(0, -1);
  }
  void wall_following(){

    double comp_p = p * err.data;
    double comp_i = i * std::accumulate(err_reading.begin(), err_reading.end(), 0);
    double comp_d = d * (err.data - prev_err.data); 
    alinhamento = 2 * p * alinhamento;

    // RCLCPP_INFO(this->get_logger(), "err = %f", err);


    angular = alinhamento
              + comp_p
              + comp_i
              + comp_d
              
              ;

  

    angular = std::clamp(angular, -1.5, 1.5);

    
    publish_velocity(linear, angular);

    err_pub_->publish(err);
    prev_err_pub_->publish(prev_err);
    sonar1_msg.data = sonar1_filtered[4];
    sonar1_pub_->publish(sonar1_msg);
    sonar2_msg.data = sonar2_filtered[4];
    sonar2_pub_->publish(sonar2_msg);
  }
  /**
   * @brief Loop de controle executado a 20 Hz.
   *
   * Implementa a lógica de navegação: verifica objetivo, calcula erros,
   * decide entre virar, andar ou parar.
   */

  void turn(){
    publish_velocity(linear, 0);
  }
  void control_loop() override
  {
    if (!has_goal()) {
      stop();
      return;
    }

    if(direita){
      indice_sonar1 = 7;
      indice_sonar2 = 8;
      indice_turn = 9;
    }
    else{
      indice_sonar1 = 0;
      indice_sonar2 = 15;
      indice_turn = 14;
    }

    angle_err = angle_to_goal();

    double front_dist = get_front_distance();

    sonar1[iterator] = sonar_ranges_[indice_sonar1];
    sonar1_filtered = sonar1;
    std::sort(sonar1_filtered.begin(), sonar1_filtered.end());

    sonar2[iterator] = sonar_ranges_[indice_sonar2];
    sonar2_filtered = sonar2;
    std::sort(sonar2_filtered.begin(), sonar2_filtered.end());

    if(direita){
      err.data = OBSTACLE_THRESHOLD/2 - ((sonar2_filtered[4] + sonar1_filtered[4]) / 2);
    }
    else{
      err.data = ((sonar2_filtered[4] + sonar1_filtered[4]) / 2) - OBSTACLE_THRESHOLD/2;
    }
    err_reading[iterator] = err.data;
    
    if(direita) alinhamento = sonar2_filtered[4] - sonar1_filtered[4];
    else alinhamento = sonar1_filtered[4] - sonar2_filtered[4];

    iterator = (iterator + 1) % 9;  

    // RCLCPP_INFO(this->get_logger(), "leituras: %lf %lf", sonar1_filtered[4], sonar2_filtered[4]);
    
    if(((sonar2_filtered[4] + sonar1_filtered[4]) / 2) < OBSTACLE_THRESHOLD and state == 2){
      RCLCPP_INFO(this->get_logger(), "Entrando em wall_following");
      state = 3;
    }
    else if (front_dist < OBSTACLE_DETECTION and state == 0) {
      RCLCPP_INFO(this->get_logger(), "Entrando em pre_wall_following");
      state = 2;
      // stop();
    }
    else if(sonar1_filtered[4] > OBSTACLE_THRESHOLD and state == 3){
      RCLCPP_INFO(this->get_logger(), "Entrando em turn");    
      state = 1;
    }
    else if(sonar_ranges_[indice_turn] > OBSTACLE_THRESHOLD and state == 1){
      RCLCPP_INFO(this->get_logger(), "Entrando em follow_line");    
      state = 0;
    }
            
    // RCLCPP_INFO(this->get_logger(), "%lf\n%lf\n%lf\n%lf\n%lf\n%lf\n%lf\n%lf\n%lf"
    // ,sonar1_filtered[0], sonar1_filtered[1], sonar1_filtered[2], sonar1_filtered[3], sonar1_filtered[4]
    // ,sonar1_filtered[5], sonar1_filtered[6], sonar1_filtered[7], sonar1_filtered[8]);    
   
    switch (state){
      case 3:
        wall_following();
        break;
      case 2:
        pre_wall_following();
        break;
      case 1:
        turn();
        break;
      default:
        follow_line();
    }

    prev_err = err;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlExample>());
  rclcpp::shutdown();
  return 0;
}