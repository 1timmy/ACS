#include "Main.hpp"

cv::Mat image, templat, heat_map;

void computeMovement(Images *images){
    
    float scale_factor = 0.5;
    int width = 320;
    int height = 156;
    int m_x = 6;
    int m_y = 8;
    int d_x = 4;
    int d_y = 140;

    cv::Rect roi(m_x,m_y,d_x,d_y);
    
    cv::resize( images->current_image,
                image,
                cv::Size(),
                scale_factor, scale_factor );
    
    cv::resize( images->previous_image(roi),
                templat,
                cv::Size(),
                scale_factor, scale_factor );
    
    cv::matchTemplate(image, templat, heat_map, cv::TM_CCOEFF_NORMED);
    
    double val_min, val_max;
    cv::Point loc_min, loc_max;
    
    cv::minMaxLoc(heat_map, &val_min, &val_max, &loc_min, &loc_max);
    
    int travel = (loc_max.x/scale_factor) - m_x;
    images->travel = travel*4;

    cv::Rect e_roi(0,58,320,40);
    
    cv::resize( images->current_image,
                image,
                cv::Size(),
                scale_factor, 1 );
    
    cv::resize( images->pattern_image(e_roi),
                templat,
                cv::Size(),
                scale_factor, 1 );
    
    cv::matchTemplate(image, templat, heat_map, cv::TM_CCOEFF_NORMED);
    cv::minMaxLoc(heat_map, &val_min, &val_max, &loc_min, &loc_max);
    int shift = loc_max.y - 58;

    images->shift = shift * 4;
    
}

bool loaded = false;

bool getMovement(Images *local_set){
    bool success = false;

    std::chrono::time_point<std::chrono::system_clock> start_comp;
    std::chrono::time_point<std::chrono::system_clock> end_comp;

    std::chrono::milliseconds dF = std::chrono::duration_cast<std::chrono::milliseconds>(local_set->c_stamp-local_set->p_stamp);
    
    start_comp = std::chrono::system_clock::now();
    ///////////////////////////////////////////////////////////////////////////////////////
    if (loaded && dF.count() <=300){
        computeMovement(local_set);
        success = true;
    }
    local_set->previous_image = local_set->current_image.clone();
    local_set->p_stamp = local_set->c_stamp;
    loaded = true;
    ///////////////////////////////////////////////////////////////////////////////////////
    end_comp = std::chrono::system_clock::now();
    std::chrono::milliseconds dV = std::chrono::duration_cast<std::chrono::milliseconds>(end_comp-start_comp);

    //std::cout << " dF: " << dF.count() << " dV: " << dV.count() << " ";
    return success;
}


/*
std::vector<std::string> getPatterns(){
    std::vector<std::string> filenames;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir ("patterns/")) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            if (ent->d_name[0] != '.'){
                filenames.push_back(ent->d_name);
            }
        }
        closedir (dir);
    }
    return filenames;
}
*/