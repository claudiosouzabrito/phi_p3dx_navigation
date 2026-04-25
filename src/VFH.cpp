#include "navigation_node.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class ControlExample : public NavigationNode
{
public:
ControlExample() : NavigationNode("VFH_cpp") {}

const int setors = 36;           
const double OBSTACLE_THRESHOLD = 1.5;  
const double linear = 0.2;   
int final_sector;
double angle;
int left_border;
int right_border;
int s_max = 10; //talves

private:
    void control_loop() override
    {
        if (!has_goal()) {
            stop();
            return;
        }

        double dist_to_goal = distance_to_goal();
        if (dist_to_goal < 0.2) {
            clear_goal();
            stop();
            return;
        }

        std::vector<int> histogram(setors, 0); 
        
        for (size_t i = 0; i < laser_ranges_.size(); ++i) {
            double dist = laser_ranges_[i];
            if (dist < OBSTACLE_THRESHOLD) {
              int indice = (i * setors / laser_ranges_.size());
              histogram[indice] = 1; 
              if (indice > 0) histogram[indice - 1] = 1;
              if (indice < setors - 1) histogram[indice + 1] = 1;
            }
        }

        std::vector<int> free_sectors;
        for (int i = 0; i < setors; ++i) {
            if (histogram[i] == 0) {
                free_sectors.push_back(i);
            }
        }

        double angle_err = angle_to_goal(); 
        
        // RCLCPP_INFO(this->get_logger(), "angle_min = %f", laser_angle_min_);
        double angle_range = laser_angle_max_ - laser_angle_min_;
        int target_sector = static_cast<int>((angle_err - laser_angle_min_) * setors / angle_range);
        target_sector = std::clamp(target_sector, 0, setors - 1);

        if (histogram[target_sector] == 0) {
            final_sector = target_sector;
        } else {
            int min_dist = setors;
            for (int i : free_sectors) {
                int d = std::abs(i - target_sector);
                if (d < min_dist) {
                    min_dist = d;
                    final_sector = i;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "finalsector antes: %d", final_sector);

        left_border = final_sector;
        right_border = final_sector;
        while(right_border > 0 and histogram[right_border - 1] == 0) right_border--;
        while(left_border < (setors - 1) and histogram[left_border + 1] == 0) left_border++;
        RCLCPP_INFO(this->get_logger(), "left: %d, right: %d", left_border, right_border);

        if(left_border - right_border <= s_max){
          final_sector = (left_border + right_border) / 2;  
          RCLCPP_INFO(this->get_logger(), "estreito: finalsector depois: %d", final_sector);
        }
        else{
          if (abs(final_sector - right_border) < abs(final_sector - left_border)) {
              final_sector = right_border + (s_max / 2);
              RCLCPP_INFO(this->get_logger(), "Vale Largo (Borda Direita): Novo final %d", final_sector);
          } 
          else {
              final_sector = left_border - (s_max / 2);
              RCLCPP_INFO(this->get_logger(), "Vale Largo (Borda Esquerda): Novo final %d", final_sector);
          }
        } 

        angle = final_sector * (angle_range / setors) + laser_angle_min_ ;
        RCLCPP_INFO(this->get_logger(), "angulo: %f\n\n", angle);

        publish_velocity(linear, angle);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlExample>());
    rclcpp::shutdown();
    return 0;
}