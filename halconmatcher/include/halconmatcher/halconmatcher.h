#include <halconcpp/HalconCpp.h>

class HalconMatcher{

    public:
        explicit HalconMatcher(void){};
        void load_scene(HalconCpp::HObjectModel3D scene);
        void load_scene(std::string filename, std::string units="mm");
        void init_model(std::string filename, float sampling_distance=0.03, std::string units="mm");
        void match();
        
    private:
        HalconCpp::HObjectModel3D reference_object;
        HalconCpp::HSurfaceModel reference_surface;
        HalconCpp::HObjectModel3D scene;

};