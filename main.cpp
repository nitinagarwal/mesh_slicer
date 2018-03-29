#if defined(NANOGUI_GLAD)
#if defined(NANOGUI_SHARED) && !defined(GLAD_GLAPI_EXPORT)
#define GLAD_GLAPI_EXPORT #endif
#include <glad/glad.h>
#else
#if defined(__APPLE__)
#define GLFW_INCLUDE_GLCOREARB
#else
#define GL_GLEXT_PROTOTYPES
#endif
#endif
#endif

#define GLEW_STATIC
#include <GL/glew.h>                /* always before GLFW */
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <nanogui/nanogui.h>
#include <igl/readOFF.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>


#include "./utils/Shader.h"
#include "./utils/WriteBMP.h"
#include "./utils/Quaternion.h"
using std::cout;
using std::endl;
using namespace nanogui;

/* mesh input */ 
Eigen::MatrixXd inputV1;
Eigen::MatrixXi inputF1;
Eigen::MatrixXd inputC1; 
const char* mesh_pathname;
std::string storage_pathname;

Screen *screen = NULL;
GLFWwindow* window = NULL;

// settings
const unsigned int SCR_WIDTH = 1500;
const unsigned int SCR_HEIGHT = 1000;

/* camera */ 
glm::vec3 cameraPos   = glm::vec3(0.0f, 0.0f,  3.0f); 
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f); 
glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f,  0.0f);

bool firstMouse = true;
float yaw   = -90.0f;	  // yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch =  0.0f;
float lastX =  1500.0f / 2.0;
float lastY =  1000.0 / 2.0;
float fov   =  45.0f;

// timing
float deltaTime = 0.0f; // time between current frame and last frame
float lastFrame = 0.0f;

//Default Values of Variables
double normal_x = 1.0;
double normal_y = 0.0;
double normal_z = 0.0;
Eigen::Vector3d old_normal; 
Eigen::Vector3d new_normal;
Eigen::Vector3d rot_axis;
GLfloat rot_angle = 0.0f; 
int num_slices = 0;
double dis_slices = 100.0;
double plane_xcord = 0; // default transations
double plane_ycord = 0;
double plane_zcord = 0; 
double plane_offset = 0; 
bool contour = false;
bool image = false;
bool color= false;

/* Cutting Plane coorinate */
double max_value,min_value;
double plane[] = {      // default plane 
    -0.01f,  -1.0f, 1.0f,  
    -0.01f, -1.0f, -1.0f, 
    -0.01f, 1.0f, 1.0f,  
    -0.01f,  1.0f, -1.0f
};
double temp_plane[12]; // creating a copy for modification
unsigned int planeind[] = {
    0, 1, 2,
    1, 3, 2
};

//Default drawing bool values
bool draw_mesh = true; // start with true;
bool draw_wire = false;
bool draw_plane = false;
bool draw_contour = false;
bool allow_cursor_movement = false;

/* for input mesh */
struct Vertex {
    glm::vec3 Position;    // Position
    glm::vec3 vertexColor;    // Vertex color
};

/* contour */
std::vector<Eigen::RowVector3d> outputVertices;
std::vector<Eigen::RowVector2d> outputEdges;
std::vector<Eigen::RowVector3d> outputColor;
std::vector<Vertex> slices;

unsigned int VBO[3], VAO[3], EBO[2]; // buffers

// function declarations
void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void glfw_cursor_callback(GLFWwindow* window, double xpos, double ypos);
void glfw_drop_callback(GLFWwindow* window, int count, const char **filenames);
void glfw_char_callback(GLFWwindow* window, unsigned int codepoint);
void glfw_scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void glfw_mouse_callback(GLFWwindow* window, int button, int action, int modifiers);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
bool point_onPlane(Eigen::Vector3d &vec1,Eigen::Vector3d &nomPL, Eigen::Vector3d &ptPL);
bool plane_triangleIntersect(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2, Eigen::Vector3d &vec3, Eigen::Vector3d &nomPL, Eigen::Vector3d &ptPL);
std::vector<Eigen::Vector3d> plane_edgeIntersect(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2, Eigen::Vector3d &vec3, Eigen::Vector3d &nomPL, Eigen::Vector3d &ptPL);
void compute_intersection(Eigen::MatrixXd &mesh_ver, Eigen::MatrixXi &mesh_fac, Eigen::MatrixXd &mesh_col, Eigen::Vector3d &normal_PL, Eigen::Vector3d &point_PL);
void writePLY(std::vector<Eigen::RowVector3d> VERTICES, std::vector<Eigen::RowVector3d> COLOR, std::vector<Eigen::RowVector2d> EDGES,std::string directory);
std::vector<Eigen::Vector3d> rayBB_intersection(Eigen::Vector3d &min_BB, Eigen::Vector3d &max_BB, Eigen::Vector3d &ray_direction);
void updatePlane();
void updatePoints(std::vector<Eigen::RowVector3d> &outputVertices, Eigen::Vector4d &size);
void updateBuffer();


int main(int argc, char ** argv){

    if(argc != 3){
        cout << "Not enough arguments" << endl;
        cout << "Usage: mesh_slicer [mesh_file_name.off] [storage_directory_location]" << endl;
        return 1;
    }else{

        mesh_pathname = argv[1];
        storage_pathname = argv[2];
    }

    /* ------------------------------Mesh Data to Load------------------------------------------------------ */
    igl::readOFF(mesh_pathname,inputV1, inputF1, inputC1);

    cout << " INPUT =====================================" << endl;
    cout << " mesh_dir = " << mesh_pathname << endl;
    cout << " storage_dir = " << storage_pathname << endl;
    cout << " #Vertices in input_mesh = " << inputV1.rows() << endl;
    cout << " #Faces in input_mesh = " << inputF1.rows() << endl;

    if (inputC1.rows() > 0){
        cout << " Vertices of input_mesh have color " << endl;
        color = true;
    }else{
        cout << " Vertices of input_mesh have NO color " << endl;
    }
    cout << " =====================================" << endl;

    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;

    min_value = inputV1.minCoeff(); // compute the overall min as we dont want to uniformaly scale the mesh down
    max_value = inputV1.maxCoeff();

    /* Walk through each of the mesh's vertices */
    for(GLuint i = 0; i < inputV1.rows(); i++)
    {
        Vertex vertex;
        glm::vec3 vec;

        // Positions --- Normalize each vertex
        vec.x = 2 * (( inputV1(i,0) - min_value)/(max_value-min_value)) -1;
        vec.y = 2 * (( inputV1(i,1) - min_value)/(max_value-min_value)) -1;
        vec.z = 2 * (( inputV1(i,2) - min_value)/(max_value-min_value)) -1;
        vertex.Position = vec;

        if( color == true ){
            // VertexColor
            vec.x = inputC1(i,0);
            vec.y = inputC1(i,1);
            vec.z = inputC1(i,2);
            vertex.vertexColor = vec;
        }else{

            vertex.vertexColor = glm::vec3(1.0f, 0.0f, 1.0f ); // magenta color mesh
        }
        vertices.push_back(vertex);
    } 
    // Walk through each of the mesh's faces
    for(GLuint i = 0; i < inputF1.rows(); i++)
    {
        indices.push_back(inputF1(i,0));
        indices.push_back(inputF1(i,1));
        indices.push_back(inputF1(i,2));
    }
    // initialize temp_plane
    for (int i = 0; i < 12; ++i) {
        temp_plane[i]=plane[i];
    }

    /* initialize GLFW context */
    glfwInit();

    glfwSetTime(0);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);   /* Needed to work on mac */
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    glfwWindowHint(GLFW_SAMPLES, 0);
    glfwWindowHint(GLFW_RED_BITS, 8);
    glfwWindowHint(GLFW_GREEN_BITS, 8);
    glfwWindowHint(GLFW_BLUE_BITS, 8);
    glfwWindowHint(GLFW_ALPHA_BITS, 8);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

    // Create a GLFWwindow object
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Mesh Slicer", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    /* intializing GLEW */
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        std::cout << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

#if defined(NANOGUI_GLAD)
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
        throw std::runtime_error("Could not initialize GLAD!");
    glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM
#endif

    /* capture mouse */
    /* glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); */

    /* ------------------Nano Gui Setting---------------------------------------------------------------- */
    // Create a nanogui screen and pass the glfw pointer to initialize
    screen = new Screen();
    screen->initialize(window, true);

    // Create nanogui gui
    FormHelper *gui = new FormHelper(screen);
    gui->setFixedSize(Eigen::Vector2i(60,20));
    ref<Window> nanoguiWindow = gui->addWindow(Eigen::Vector2i(1,1), "Mesh Slicer");

    gui->addGroup("Input (Plane Settings)");
    gui->addVariable("Normal Vector (x-dir)", normal_x)->setSpinnable(true);
    gui->addVariable("Normal Vector (y-dir)", normal_y)->setSpinnable(true);
    gui->addVariable("Normal Vector (z-dir)", normal_z)->setSpinnable(true);

    gui->addVariable("Position (xcord)", plane_xcord)->setSpinnable(true);
    gui->addVariable("Position (ycord)", plane_ycord)->setSpinnable(true);
    gui->addVariable("Position (zcord)", plane_zcord)->setSpinnable(true);
    /* gui->addVariable("Offset", plane_offset)->setSpinnable(true); */
    gui->addVariable("# of Slices", num_slices)->setTooltip("Number of slices. Either specify the number of slices or the interslice distance."); 
    gui->addVariable("Inter-Slice Distance", dis_slices)->setTooltip("Interslice Distance. Either specify the interslice distance or the number of slics.");


    gui->addGroup("Output");
    gui->addVariable<bool>("Contour", [&](bool c_temp){
            contour = c_temp;
            },[&](){
            return false;
            })->setTooltip("Output is a .PLY file with a list of vertices and edges of the intersecting contours.");

    gui->addVariable<bool>("Image", [&](bool c_img){
            image = c_img;
            },[&](){
            return false;
            })->setTooltip("Output is a rasterized image of the intersecting contours");

    gui->addButton("Generate", []() {

            double intersectDistance, inter_sliceDist;
            int number_slices;

            if(contour && image){
            cout << "ERROR:------Choose Only One Output--------" << endl;
            }else if(!contour && !image){
            cout << "ERROR:------Choose At Least One Output--------" << endl;
            }else if(num_slices ==0 && dis_slices ==0){
            cout << "ERROR:------BOTH # of slices and distance between slices can not be ZERO--------" << endl;
            }else if(num_slices !=0 && dis_slices != 0){
            cout << "ERROR:------BOTH # of slices and distance between slices can not be NONZERO--------" << endl;
            }else if(num_slices == 1){
            cout << "ERROR:------# of slices should be greater than 1--------" << endl;
            }else{

            /* check which two pts does the ray (pt with new normal) intersect with the BB */
            /* can be done using the min and max of the Bounding box only */
            Eigen::Vector3d BB_min, BB_max;
            BB_min = inputV1.colwise().minCoeff();
            BB_max = inputV1.colwise().maxCoeff();

            BB_min = BB_min.array() + 5;   //slightly shrinking the bounding box to allow intersection 
            BB_max = BB_max.array() - 5;
            std::vector<Eigen::Vector3d> intersectPts;
            Eigen::Vector3d new_point;

            intersectPts = rayBB_intersection(BB_min, BB_max, new_normal);

            intersectDistance = sqrt( pow(intersectPts.at(0)(0)-intersectPts.at(1)(0),2) + pow(intersectPts.at(0)(1)-intersectPts.at(1)(1),2) + pow(intersectPts.at(0)(2)-intersectPts.at(1)(2),2) );  // distance

            if(num_slices ==0){
                inter_sliceDist = dis_slices/4;              // the ratio 100um -> 25um
                number_slices = intersectDistance / inter_sliceDist;
            }else{
                number_slices = num_slices;
                inter_sliceDist = intersectDistance / (number_slices-1);
            }

            if(contour){
                cout << "Generating contours----" << endl;
            }else{
                cout << "Generating images-----" << endl;
            }

            for(int i=0; i<number_slices; i++){    // generating the contours or images

                new_point = intersectPts.at(0) + (i*inter_sliceDist)*new_normal;
                compute_intersection(inputV1, inputF1, inputC1, new_normal,new_point);     // slice the mesh

                if(outputVertices.size()>0){ // there might be no vertices at all from the intersection

                    cout << "contour # " << i+1 << endl;
                    if(contour){

                        writePLY(outputVertices, outputColor, outputEdges, storage_pathname + "/" + std::to_string(i+1)+".ply");

                    }else{

                        Eigen::Vector4d sz;
                        updatePoints(outputVertices,sz);  // aligning the vertices to z axis for writing to bmp
                        WriteBMP(ceil(sz(0))+50, ceil(sz(1))+50, outputVertices, outputColor, outputEdges, storage_pathname + "/" + std::to_string(i+1)+".bmp");
                    }

                }

                outputVertices.clear();  // clearing the old values
                outputColor.clear();
                outputEdges.clear();
            }
            cout << "Done----" << endl;
            }
    });

    gui->addGroup("GUI");
    gui->addButton("Slice the Mesh", []() { 
            outputVertices.clear();  // clearing the old values
            outputColor.clear();
            outputEdges.clear();
            slices.clear();

            updatePlane(); // update the location of the plane

            Eigen::Vector3d PT;
            PT << temp_plane[0], temp_plane[1], temp_plane[2]; // take any point on the plane
            PT = ((PT.array()+1)*(max_value-min_value))/2 + min_value; // renormalize

            // slice the mesh
            compute_intersection(inputV1, inputF1, inputC1, new_normal,PT); 

            /* Walk through each of the contour's vertices */
            for(GLuint i = 0; i < outputVertices.size(); i++)
            {
            Vertex v1;
            glm::vec3 v2;
            // Positions --- Normalize each vertex
            v2.x = 2 * (( outputVertices.at(i)(0) - min_value)/(max_value-min_value)) -1;
            v2.y = 2 * (( outputVertices.at(i)(1) - min_value)/(max_value-min_value)) -1;
            v2.z = 2 * (( outputVertices.at(i)(2)  - min_value)/(max_value-min_value)) -1;
            v1.Position = v2;

            if( color == true ){
                // VertexColor
                v2.x = outputColor.at(i)(0);
                v2.y = outputColor.at(i)(1);
                v2.z = outputColor.at(i)(2);
                v1.vertexColor = v2;
            }else{
                v1.vertexColor = glm::vec3(1.0f, 0.0f, 1.0f ); // magenta color mesh
            }
            slices.push_back(v1);
            } 

            updateBuffer();

            //draw contour
            draw_contour = true;
            draw_wire = false;
            draw_mesh = false;
            draw_plane = false; 
            cout << "Slice pressed." << endl;  
    });

    gui->addButton("Reload Mesh", []() { 
            draw_mesh = true; 
            draw_wire = true; 
            draw_plane = false; 
            draw_contour = false; 
            /* rot_axis = old_normal; // resetting the plane orientation */
            cout << "Mesh Reloaded" << endl;  
            });

    screen->setVisible(true);
    screen->performLayout();
    /* nanoguiWindow->center(); */

    /* ------------------Call back Functions---------------------------------------------------------------- */

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glfwSetCursorPosCallback(window, glfw_cursor_callback);
    glfwSetScrollCallback(window, glfw_scroll_callback);
    glfwSetMouseButtonCallback(window, glfw_mouse_callback);
    glfwSetKeyCallback(window, glfw_key_callback);
    glfwSetCharCallback(window, glfw_char_callback);
    glfwSetDropCallback(window, glfw_drop_callback);

    /* ------------------OPENGL shader configurations ------------------------------------------------------ */

    /* Build and compile the shader program */
    Shader ourProgram("../misc/vertex_shader","../misc/fragment_shader");
    Shader Wireframe("../misc/vertex_shader","../misc/fragment_shader_wire");
    Shader PlaneShader("../misc/vertex_shader_plane","../misc/fragment_shader_plane");
    Shader ContourShader("../misc/vertex_shader_contour","../misc/fragment_shader");

    /* ------------------------------Init VAO, VBO, EBO----------------------------------------------------- */

    glGenVertexArrays(3, VAO);
    glGenBuffers(3, VBO);
    glGenBuffers(2, EBO);

    glBindVertexArray(VAO[0]);

    glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex) , &vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);

    /* position attribute */
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    /* color attribute */
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex,vertexColor));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    /* Unbinding VAO so that other VAO can be initialized */
    glBindVertexArray(0); 

    /* ----------------second object - plane---------------------------------------------------- */

    glBindVertexArray(VAO[1]);

    glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(plane), plane, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(planeind), planeind, GL_STATIC_DRAW);

    /* position attribute */
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(double), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0); 

    old_normal << normal_x, normal_y, normal_z; 
    rot_axis = old_normal;

    /* ------------------------------Game Loop-------------------------------------------------------------- */
    while (!glfwWindowShouldClose(window)) {

        /* Getting the time for camera movement */
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_PROGRAM_POINT_SIZE); // can change the pt size 

        /* event handling */
        glfwPollEvents();

        glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /* computing the angle of rotation for plane */
        new_normal<< normal_x, normal_y, normal_z; 
        new_normal.normalize();
        old_normal.normalize();
        if(old_normal.dot(new_normal)<=1 && old_normal.dot(new_normal)>=-1 && old_normal != new_normal){
            rot_angle = acos(old_normal.dot(new_normal));
            rot_axis = old_normal.cross(new_normal);
        }else if(old_normal==new_normal){
            rot_angle = 0;
            rot_axis = old_normal;
        }

        /* Camera View and Model Transformation */
        glm::mat4 view;
        view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);

        glm::mat4 model(1.0f); // not changing the model
        model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f));

        glm::mat4 projection;
        projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 10.0f);

        if(draw_contour){
            // draw the sliced mesh. i.e contour
            ContourShader.Use();         /* glUseProgram(shaderProgram); */
            ContourShader.setMat4("view", view);
            ContourShader.setMat4("model", model);
            ContourShader.setMat4("projection", projection);

            glBindVertexArray(VAO[2]);
            glDrawArrays(GL_POINTS,0, slices.size());
            glBindVertexArray(0);
        }

        if(draw_plane){
            /* plane overlay */
            PlaneShader.Use();
            glm::mat4 model_plane;
            model_plane = glm::translate(model_plane, glm::vec3(plane_xcord,plane_ycord,plane_zcord));
            model_plane = glm::rotate(model_plane, rot_angle , glm::vec3(rot_axis(0),rot_axis(1),rot_axis(2)));
            PlaneShader.setMat4("model", model_plane);
            PlaneShader.setMat4("projection", projection);
            PlaneShader.setMat4("view", view);

            glBindVertexArray(VAO[1]);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);        // unbinding 
        }

        if(draw_mesh){
            /* drawing the mesh */ 
            ourProgram.Use();         /* glUseProgram(shaderProgram); */
            ourProgram.setMat4("view", view);
            ourProgram.setMat4("model", model);
            ourProgram.setMat4("projection", projection);

            glBindVertexArray(VAO[0]);
            glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }

        if(draw_wire){
            /* overlay wireframe */
            Wireframe.Use();
            Wireframe.setMat4("view", view);
            Wireframe.setMat4("model", model);
            Wireframe.setMat4("projection", projection);

            glBindVertexArray(VAO[0]);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glEnable(GL_POLYGON_OFFSET_LINE);
            glPolygonOffset(-1,-1);
            glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
            glDisable(GL_POLYGON_OFFSET_LINE);
            glBindVertexArray(0);        
        }

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // Draw nanogui
        screen->drawContents();
        screen->drawWidgets();

        glfwSwapBuffers(window);
    }
    // Terminate GLFW, clearing any resources allocated by GLFW.
    glfwTerminate();
    return 0;
}


void updateBuffer(){

    glBindVertexArray(VAO[2]);

    glBindBuffer(GL_ARRAY_BUFFER, VBO[2]);
    glBufferData(GL_ARRAY_BUFFER, slices.size() * sizeof(Vertex) , &slices[0], GL_DYNAMIC_DRAW);

    /* position attribute */
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    /* color attribute */
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex,vertexColor));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0); 
}

void updatePlane(){

    Eigen::Matrix<double,4,4>  pt_plane;
    Eigen::Matrix<double, 4, 4> PL;
    PL << plane[0], plane[1], plane[2], 1,
       plane[3], plane[4], plane[5], 1,
       plane[6], plane[7], plane[8], 1,
       plane[9], plane[10], plane[11], 1;

    Eigen::Matrix<double, 4, 4> Tran; 
    Eigen::Matrix<double, 4, 4> Tran2; 
    Eigen::Matrix<double, 4, 4> R; 

    Tran.setIdentity(4,4);
    R.setIdentity(4,4);
    Tran2.setIdentity(4,4);

    Quaternion quat_plane(rot_axis, rot_angle);
    R = quat_plane.Quat_to_Rotmatrix();

    double xcenter = (PL(0,0) + PL(3,0))/2;
    double ycenter = (PL(0,1) + PL(3,1))/2;
    double zcenter = (PL(0,2) + PL(3,2))/2;

    Tran.col(3) << xcenter, ycenter, zcenter, 1;
    Tran2.col(3) << plane_xcord, plane_ycord, plane_zcord;

    pt_plane = Tran2 * Tran *  R  * Tran.inverse() * PL.transpose();
    pt_plane.transposeInPlace();

    /* update the plane */
    temp_plane[0] = pt_plane(0,0); 
    temp_plane[1] =pt_plane(0,1); 
    temp_plane[2] = pt_plane(0,2); 

    temp_plane[3] =pt_plane(1,0); 
    temp_plane[4] =pt_plane(1,1);
    temp_plane[5]= pt_plane(1,2); 

    temp_plane[6]= pt_plane(2,0); 
    temp_plane[7]= pt_plane(2,1); 
    temp_plane[8]= pt_plane(2,2); 

    temp_plane[9] = pt_plane(3,0); 
    temp_plane[10] = pt_plane(3,1); 
    temp_plane[11] = pt_plane(3,2); 
}


void compute_intersection(Eigen::MatrixXd &mesh_ver, Eigen::MatrixXi &mesh_fac, Eigen::MatrixXd &mesh_col, Eigen::Vector3d &normal_PL, Eigen::Vector3d &point_PL){

    std::vector <Eigen::Vector3d> vertices_temp;

    int j=0; // counter for vertices and color
    int k=0; // counter for the edges

    /* check whether triangles intersect the plane or not */
    for (int i = 0; i < mesh_fac.rows(); i++) {

        Eigen::Vector3d vec1 = mesh_ver.row(mesh_fac(i,0)); // vertices of triangle faces
        Eigen::Vector3d vec2 = mesh_ver.row(mesh_fac(i,1));
        Eigen::Vector3d vec3 = mesh_ver.row(mesh_fac(i,2));

        if( plane_triangleIntersect(vec1,vec2,vec3,normal_PL,point_PL) ){
            /* there is an intersection */

            vertices_temp = plane_edgeIntersect(vec1, vec2, vec3, normal_PL, point_PL);
            /* vertices_temp.rows() ==2 or 3 cant be anything else*/ 

            if(vertices_temp.size() ==3){

                outputVertices.push_back(vertices_temp.at(0));
                outputVertices.push_back(vertices_temp.at(1));
                outputVertices.push_back(vertices_temp.at(2));

                Eigen::Vector2d t(j, j+1);
                outputEdges.push_back(t);
                t << j+1,j+2;
                outputEdges.push_back(t);
                t << j+2,j;
                outputEdges.push_back(t);

                outputColor.push_back(mesh_col.row(mesh_fac(i,0)));
                outputColor.push_back(mesh_col.row(mesh_fac(i,0)));
                outputColor.push_back(mesh_col.row(mesh_fac(i,0)));

                j = j+3;
                k = k+3;
            }else if( vertices_temp.size() == 2 ){

                outputVertices.push_back(vertices_temp.at(0));
                outputVertices.push_back(vertices_temp.at(1));

                Eigen::Vector2d t(j, j+1);
                outputEdges.push_back(t);

                outputColor.push_back(mesh_col.row(mesh_fac(i,0)));
                outputColor.push_back(mesh_col.row(mesh_fac(i,0)));

                j = j+2;
                k = k+1;
            }
        }
    }
}

std::vector<Eigen::Vector3d> plane_edgeIntersect(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2, Eigen::Vector3d &vec3, Eigen::Vector3d &nomPL, Eigen::Vector3d &ptPL){

    /* compute the intersection of line segments (traiangle edges) with plane */
    std::vector<Eigen::Vector3d> intersects;

    /* all three pts lie on plane */
    if (point_onPlane(vec1,nomPL,ptPL) && point_onPlane(vec2,nomPL,ptPL) && point_onPlane(vec3,nomPL,ptPL) ){

        intersects.push_back(vec1);
        intersects.push_back(vec2);
        intersects.push_back(vec3);
        return intersects;
    }

    Eigen::Matrix<double,4,3> vertices; /* new matrix for looping those the edges */
    vertices.row(0) = vec1; 
    vertices.row(1) = vec2;
    vertices.row(2) = vec3; 
    vertices.row(3) = vec1;

    for (int i = 0; i < 3; i++) {

        Eigen::Vector3d v1 = vertices.row(i);
        Eigen::Vector3d v2 = vertices.row(i+1);

        if(nomPL.dot(v2-v1) == 0){
            /* edge can be on plane or parallel. checking if edge on plane(both points lie on plane)*/
            if( point_onPlane(v2,nomPL,ptPL) && point_onPlane(v1,nomPL,ptPL) ){
                intersects.push_back(vec1);
                intersects.push_back(vec2);
                return intersects;
            } 
        }else{

            double r =(nomPL.dot(ptPL-v1)) / (nomPL.dot(v2-v1));
            if( r >= 0 & r <=1 ){

                Eigen::Vector3d pt = v1 + r*(v2-v1);
                intersects.push_back(pt);
            }
        }
    }
    /* checking if the vertices are duplicate. could be if the triangle is only touching one vertex */
    if (intersects.size() == 2 && intersects[0] == intersects[1] ){
        intersects.clear();
    }
    return intersects;
}

bool plane_triangleIntersect(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2, Eigen::Vector3d &vec3, Eigen::Vector3d &nomPL, Eigen::Vector3d &ptPL){

    /* check whether the triangle cross the plane or not */
    double a = (vec1 - ptPL).dot(nomPL);
    double b = (vec2 - ptPL).dot(nomPL);
    double c = (vec3 - ptPL).dot(nomPL);

    if (a > 0 && b > 0 && c > 0){  //    /* checking if any two scalars are diff than the other */
        return false;
    }else if(a < 0 && b < 0 && c < 0){
        return false;
    }else{
        return true; // vertices can be on the plane as well
    }
}

bool point_onPlane(Eigen::Vector3d &vec1,Eigen::Vector3d &nomPL, Eigen::Vector3d &ptPL){
    /* checking if point lie on plane. satisfy plane equ */
    if ( (vec1 - ptPL).dot(nomPL) == 0 ){
        return true;
    }else{
        return false;
    }
}

void updatePoints(std::vector<Eigen::RowVector3d> &outputVertices, Eigen::Vector4d &size){
    /* rotating the points to make them align to z axis */

    double diff_deg = acos(Eigen::RowVector3d(0,0,1).dot(new_normal));  // align to z axis
    Eigen::RowVector3d axis = new_normal.cross(Eigen::RowVector3d(0,0,1));

    Eigen::MatrixXd inputV,outputV;
    Eigen::Matrix<double,4,4> T;
    Eigen::Matrix<double,4,4> R;
    T.setIdentity();

    inputV.resize(outputVertices.size(),4);
    outputV.resize(outputVertices.size(),4);

    for(int i=0; i<outputVertices.size(); i++){ // convert to eigen matrix useful to find the centroid
        inputV.row(i) << outputVertices.at(i)(0), outputVertices.at(i)(1), outputVertices.at(i)(2), 1;
    }

    Eigen::Vector4d max_contour = inputV.colwise().maxCoeff();
    Eigen::Vector4d min_contour = inputV.colwise().minCoeff();
    Eigen::Vector4d centroid = (min_contour-max_contour)/2; 

    T.col(3) << centroid(0), centroid(1), centroid(2), 1; 

    Quaternion Q(axis,diff_deg);
    R = Q.Quat_to_Rotmatrix();

    outputV = T * R * T.inverse() * inputV.transpose();

    /* translate so that all vertices are +ve with some buffer*/
    min_contour = outputV.rowwise().minCoeff();
    T.col(3) << -min_contour(0)+50, -min_contour(1)+50, -min_contour(2)+50, 1;

    outputV = T * outputV;
    outputV.transposeInPlace();

    Eigen::RowVector3d newvertices;

    for(int i=0; i<outputVertices.size(); i++){

        newvertices << outputV(i,0), outputV(i,1), outputV(i,2);
        outputVertices.at(i) = newvertices; 
    }

    /* to get the size of the final image */
    size = outputV.colwise().maxCoeff();
}


std::vector<Eigen::Vector3d> rayBB_intersection(Eigen::Vector3d &min_BB, Eigen::Vector3d &max_BB, Eigen::Vector3d &ray_direction){
    /* copmuting the intersection of the new normal with the BB */

    std::vector <Eigen::Vector3d> BB_intersects;
    Eigen::Vector3d temp;

    if (abs(ray_direction.dot(Eigen::Vector3d(1,0,0))) == 1){ // direction parallel to x axis

        BB_intersects.push_back(min_BB);

        temp << max_BB(0),min_BB(1), min_BB(2);
        BB_intersects.push_back(temp);

    }else if(abs(ray_direction.dot(Eigen::Vector3d(0,1,0))) == 1){ // direction parallel to y axis

        BB_intersects.push_back(min_BB);

        temp << min_BB(0),max_BB(1), min_BB(2);
        BB_intersects.push_back(temp);

    }else if(abs(ray_direction.dot(Eigen::Vector3d(0,0,1))) == 1){ // direction parallel to z axis 

        BB_intersects.push_back(min_BB);

        temp << min_BB(0),min_BB(1), max_BB(2);
        BB_intersects.push_back(temp);
    }else{

        double r;
        double r1 = 1000000.0;
        Eigen::Vector3d ray_pt1;
        ray_pt1 = min_BB + ray_direction;  // new pt in same direction

        if( (Eigen::Vector3d(1,0,0).dot(ray_pt1-min_BB)) != 0 ){

            r =(Eigen::Vector3d(1,0,0).dot(max_BB-min_BB)) / (Eigen::Vector3d(1,0,0).dot(ray_pt1-min_BB));
            if( r1 > r ){
                r1 = r;
            }
        }

        if( (Eigen::Vector3d(0,1,0).dot(ray_pt1-min_BB)) != 0 ){

            r =(Eigen::Vector3d(0,1,0).dot(max_BB-min_BB)) / (Eigen::Vector3d(0,1,0).dot(ray_pt1-min_BB));

            if( r1 > r ){
                r1 = r;
            }
        }

        if( (Eigen::Vector3d(0,0,1).dot(ray_pt1-min_BB)) != 0 ){

            r =(Eigen::Vector3d(0,0,1).dot(max_BB-min_BB)) / (Eigen::Vector3d(0,0,1).dot(ray_pt1-min_BB));

            if( r1 > r ){
                r1 = r;
            }
        }
        BB_intersects.push_back(min_BB);  
        BB_intersects.push_back(min_BB + r1*ray_direction);  
    }
    return BB_intersects;
}


void writePLY(std::vector<Eigen::RowVector3d> VERTICES, std::vector<Eigen::RowVector3d> COLOR, std::vector<Eigen::RowVector2d> EDGES,std::string directory){

    FILE * PLYfile = fopen(directory.c_str(),"w");

    fprintf(PLYfile, "ply\n");
    fprintf(PLYfile, "format ascii 1.0\n");
    fprintf(PLYfile, "element vertex %d\n", (int)VERTICES.size());
    fprintf(PLYfile, "property float x\n");
    fprintf(PLYfile, "property float y\n");
    fprintf(PLYfile, "property float z\n");
    fprintf(PLYfile, "property uchar red\n");
    fprintf(PLYfile, "property uchar green\n");
    fprintf(PLYfile, "property uchar blue\n");
    fprintf(PLYfile, "element faces %d\n", (int)EDGES.size());
    fprintf(PLYfile, "property list uchar int vertex_indices\n");
    fprintf(PLYfile, "end_header\n");

    for(int i=0; i<VERTICES.size();i++){

        fprintf(PLYfile, "%.3f %.3f %.3f %3.0f %3.0f %3.0f\n",VERTICES.at(i)(0), VERTICES.at(i)(1), VERTICES.at(i)(2),COLOR.at(i)(0)*255,COLOR.at(i)(1)*255,COLOR.at(i)(2)*255);
    }
    for(int i=0; i<EDGES.size();i++){

        fprintf(PLYfile, "2 %d %d\n",(int)EDGES.at(i)(0),(int)EDGES.at(i)(1));
    }
    fclose(PLYfile);
}

void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    float cameraSpeed = 2.5 * deltaTime; 
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE); //     closing the application

    screen->keyCallbackEvent(key, scancode, action, mode); 

    if (key == GLFW_KEY_W){
        cameraPos += cameraSpeed * cameraFront;
    }
    if (key == GLFW_KEY_S)
        cameraPos -= cameraSpeed * cameraFront;
    if (key == GLFW_KEY_A)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (key == GLFW_KEY_D)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
}

void glfw_cursor_callback(GLFWwindow* window, double xpos, double ypos)
{
    screen->cursorPosCallbackEvent(xpos, ypos);

    if(allow_cursor_movement){
        if (firstMouse)
        {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
        lastX = xpos;
        lastY = ypos;

        float sensitivity = 0.1f; // change this value to your liking
        xoffset *= sensitivity;
        yoffset *= sensitivity;

        yaw += xoffset;
        pitch += yoffset;

        // make sure that when pitch is out of bounds, screen doesn't get flipped
        if (pitch > 89.0f)
            pitch = 89.0f;
        if (pitch < -89.0f)
            pitch = -89.0f;

        glm::vec3 front;
        front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        front.y = sin(glm::radians(pitch));
        front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        cameraFront = glm::normalize(front);
    }
}

void glfw_scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    /* scroll will only work when inside the window */
    if( screen->scrollCallbackEvent(xoffset, yoffset) == false){

        if (fov >= 1.0f && fov <= 45.0f)
            fov -= yoffset;
        if (fov <= 1.0f)
            fov = 1.0f;
        if (fov >= 45.0f)
            fov = 45.0f;
    }
}

void glfw_mouse_callback(GLFWwindow* window, int button, int action, int modifiers)
{
    screen->mouseButtonCallbackEvent(button, action, modifiers); 

    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS){
        allow_cursor_movement = false;
    }
    if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS){
        allow_cursor_movement = true;
    }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void glfw_drop_callback(GLFWwindow* window, int count, const char **filenames)
{
    screen->dropCallbackEvent(count, filenames);
}

void glfw_char_callback(GLFWwindow* window, unsigned int codepoint)
{
    switch(codepoint){

        case 'P':
        case 'p':
            {
                draw_plane = !draw_plane;
                break;
            }
        case 'M':
        case 'm':
            {
                draw_wire = !draw_wire;
                break;
            }
        case 'H':
        case 'h':
            {
                cout << " ======================= HELP MENU ===========================" << endl;
                cout << " M/m : Overlay Mesh  " << endl;
                cout << " P/p : Show the Slicing Plane " << endl;
                cout << " Esc : Quit Application " << endl;
                cout << " W/S/A/D : Move front, back, left & right " << endl;
                cout << " Mouse_Right_Click : Allow Camera Movement " << endl;
                cout << " Mouse_Left_Click : Stop Camera Movement " << endl;
                cout << " ======================= HELP MENU ===========================" << endl;
            }
        default:
            break;
    } 
    screen->charCallbackEvent(codepoint);
}






