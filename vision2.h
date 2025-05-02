
// representation of a labeled blob (robot marker or obstacle)
class Item {
public:
    int label;
    int ic, jc, area;
    double R, G, B;
    Item(int label, int ic, int jc, int area, double R, double G, double B);
};

// delete all Item pointers in array
void reset_items(Item* items[], int nlabels);

// Extract blob features: centroids, area, color averages
void features(
    image& grey,
    image& rgb,
    image& label,
    int n_labels,
    int label_number[],
    double ic[],
    double jc[],
    double area[],
    double R_ave[],
    double G_ave[],
    double B_ave[]
);

// binary mask creation
void build_black_mask(image& rgb, image& mask_black, image& temp_grey);
void remove_small_areas(image& grey, image& label, double count[], int min_area);
void build_color_mask(image& rgb, image& color_mask, image& temp_grey);
void combine_masks(image& mask, image& color_mask, image& black_mask);

// set robot marker centroids to variables in main
void set_robot_centroids(
    int& gx, int& gy,
    int& rx, int& ry,
    int& ox, int& oy,
    int& bx, int& by,
    Item* items[],
    int nlabels
);


void set_obstacle_centroids_and_radii(double obs_ic[], double  obs_yc[], double obs_r[], Item* items[], int nlabels, int& obs_count); 