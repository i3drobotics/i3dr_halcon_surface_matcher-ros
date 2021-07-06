#include <halconcpp/HalconCpp.h>
#include <vector>

class HalconMatcher{

    struct found_object{
        HalconCpp::HObjectModel3D points;
        double score;
        std::vector<double> pose;
    };

    public:
        HalconMatcher(void);
        void load_scene(HalconCpp::HObjectModel3D scene);
        void load_scene(std::string filename, std::string units="mm");
        void init_model(std::string filename, float sampling_distance=0.03, std::string units="mm");
        void match();
        void match_vanilla();
        void cleanup();

        std::vector<found_object> objects;

    private:
        HalconCpp::HObjectModel3D reference_object;
        HalconCpp::HSurfaceModel reference_surface;
        HalconCpp::HObjectModel3D scene;

        

};