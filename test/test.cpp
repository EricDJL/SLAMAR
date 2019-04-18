#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

using namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

const GLchar *vertexShaderSource = "#version 330 core\n"
                                 "layout (location = 0) in vec3 position;\n"
                                 "layout (location = 1) in vec3 color;\n"
                                 "out vec3 ourcolor;\n"
                                 "void main()\n"
                                 "{\n"
                                 " gl_Position = vec4(position,1.0);\n"
                                 " ourcolor = color;\n"
                                 "}\0";

const GLchar *fragmentShaderSource = "#version 330 core\n"
                                   "in vec3 ourcolor;\n"
                                   "out vec4 color;\n"
                                   "void main()\n"
                                   "{\n"
                                   " color = vec4(ourcolor,1.0f);\n"
                                   "}\n\0";

int main() {
    // glfw 初始化和配置
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH,SCR_HEIGHT,"计算机图形学-homework1",NULL,NULL);
    if(window == NULL){
        cout<<"Failed to create glfw window"<<endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window,framebuffer_size_callback);

    // 初始化GLAD load all opengl function pointers
    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
        cout<<"Failed to initialize GLAD"<<endl;
        return -1;
    }

    // 构建编译着色器程序
    // vertex shader
    int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader,1,&vertexShaderSource,NULL);
    glCompileShader(vertexShader);
    // 检查着色器编译是否正确
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader,GL_COMPILE_STATUS,&success);
    if(!success){
        glGetShaderInfoLog(vertexShader,512,NULL,infoLog);
        cout<<"ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"<<infoLog<<endl;
    }

    // fragment shader
    int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader,1,&fragmentShaderSource,NULL);
    glCompileShader(fragmentShader);
    //检查是否有编译错误
    glGetShaderiv(fragmentShader,GL_COMPILE_STATUS,&success);
    if(!success){
        glGetShaderInfoLog(fragmentShader,512,NULL,infoLog);
        cout<<"ERROE::SHADER::FRAGMENT::COMPILATION_FAILED\n"<<infoLog<<endl;
    }
    // link shaders
    int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram,vertexShader);
    glAttachShader(shaderProgram,fragmentShader);
    glLinkProgram(shaderProgram);
    // 检查是否有链接错误
    glGetProgramiv(shaderProgram,GL_LINK_STATUS,&success);
    if(!success){
        glGetProgramInfoLog(shaderProgram,512,NULL,infoLog);
        cout<<"ERROR::SHADER::PROGRAM::LINKING_FAILED\n"<<infoLog<<endl;
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // 设置顶点数据(buffer)并且配置顶点属性
    float vertices[] ={
            // Positions         // Colors
            0.0f,-0.5f,0.0f, 1.0f,0.0f,0.0f,  // 1 Bottom Right
            -0.5f,-0.5f,0.0f, 1.0f,0.0f,0.0f,  // 1 Bottom Left
            0.0f,0.0f,0.0f,  1.0f,0.0f,0.0f,   // 1 Top Right
            -0.5f,0.0f,0.0f,  1.0f,0.0f,0.0f,   // 1 Top Left
            0.0f,0.0f,0.0f, 0.0f,0.0f,1.0f,  // 2 Bottom Right
            -0.5f,0.0f,0.0f, 0.0f,0.0f,1.0f,  // 2 Bottom Left
            0.3f,0.3f,0.0f, 0.0f,0.0f,1.0f,   // 2 Top Right
            -0.2f,0.3f,0.0f, 0.0f,0.0f,1.0f,  // 2 Top Left
            0.3f,-0.2f,0.0f, 0.0f,1.0f,0.0f,  // 3 Bottom Right
            0.0f,-0.5f,0.0f, 0.0f,1.0f,0.0f,  // 3 Bottom Left
            0.3f,0.3f,0.0f,  0.0f,1.0f,0.0f,   // 3 Top Right
            0.0f,0.0f,0.0f,  0.0f,1.0f,0.0f   // 3 Top Left
    };

    // 绘制矩形的顺序
    unsigned int indices[] = {  // note that we start from 0!
            0, 1, 3,  // 1 Triangle
            0, 2, 3,  // 2 Triangle
            4, 5, 7,  // 3 Triangle
            4, 6, 7,   // 4 Triangle
            8, 9, 11,  // 5 Triangle
            8, 10, 11   // 6 Triangle
    };

    unsigned int VBO,VAO,EBO;
    glGenVertexArrays(1,&VAO);
    glGenBuffers(1,&VBO);
    glGenBuffers(1,&EBO);
    // 首先绑定VAO对象　再绑定VBO对象　然后配置顶点属性
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER,VBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(vertices),vertices,GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,sizeof(indices),indices,GL_STATIC_DRAW);

    // 配置顶点属性　0
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(GLvoid*)0);
    glEnableVertexAttribArray(0);
    // 配置顶点属性　1
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(GLvoid*)(3 * sizeof(GLfloat)));
    glEnableVertexAttribArray(1);

    // unbind VBO
    glBindBuffer(GL_ARRAY_BUFFER,0);
    // unbind VAO
    glBindVertexArray(0);


    // render loop
    while(!glfwWindowShouldClose(window)){
        // 输入
        processInput(window);

        // 渲染指令
        glClearColor(0.2f,0.3f,0.3f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // 画出一个三角形
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        //glDrawArrays(GL_TRIANGLES,0,6);
        glDrawElements(GL_TRIANGLES,18,GL_UNSIGNED_INT,0);
        // 检查并调用事件　交换缓冲
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 释放资源
    glDeleteVertexArrays(1,&VAO);
    glDeleteBuffers(1,&VBO);
    glDeleteBuffers(1,&EBO);

    // glfw terminate
    glfwTerminate();
    return 0;
}

// 处理键盘输入
void processInput(GLFWwindow *window)
{
    if(glfwGetKey(window,GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window,true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow* window,int width,int height)
{
    glViewport(0,0,width,height);
}
