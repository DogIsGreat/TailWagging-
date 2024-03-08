#include "datafile.h"
#include <stdio.h>
#include <string.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv2.h>
#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>


struct params{
    double K;
    double M;
};

int func (double t, const double y[], double f[], void *params){
    (void)(t);
    struct params *p = (struct params *)params;
    //int y[4] = {0,1,2,3};
    //int v[4] = {3,4,5,6}; 
    f[1] = (2*y[0]-y[1])*-p->K/p->M;
    f[2] = (-y[0]+2*y[1]-y[2])*-p->K/p->M;
    f[3] = (-y[1]+2*y[2])*-p->K/p->M;
    return GSL_SUCCESS;
}

int main(){
     
    int* jac = NULL;
    double t = 0.0, t1 = 100.0;
    bool pause = 0;
    double y[4] = {0,1,2,3};
    struct params p = {1,1};

    gsl_odeiv2_system odesys = {func, jac, 1, &p};
    gsl_odeiv2_driver *odedrive = gsl_odeiv2_driver_alloc_y_new(&odesys, gsl_odeiv2_step_rkf45, 1e-6,1e-6,0.0);
   
    int SCREEN_WIDTH = 1280;
    int SCREEN_HEIGHT = 720;

    // Initialize the window
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Plot Graphs");

    SetTargetFPS(60);

    // Main Graphics Loop
    while(!WindowShouldClose()){
        BeginDrawing();

        ClearBackground(RAYWHITE);

        DrawLine(SCREEN_WIDTH / 2, 0, SCREEN_WIDTH / 2, SCREEN_HEIGHT, BLACK);
        DrawLine(0, SCREEN_HEIGHT / 2, SCREEN_WIDTH, SCREEN_HEIGHT /2, BLACK);

        // Plot a sin function
        for (int x = -SCREEN_WIDTH /2; x < SCREEN_WIDTH / 2; x++){
            float y = sinf(x * 0.01f) * 100; // Sine wave function

            // Translate and scale graph points to screen coordinates
            int screenX = x + SCREEN_WIDTH / 2;
            int screenY = -y + SCREEN_HEIGHT / 2;

            DrawPixel(screenX, screenY, RED);
        }

        // Plot Runge-Kutte 45 ODE solution curve for r1
        for ( int i = 1; i <= 100; i++){
            double ti = i * t1 / 100;
            int status = gsl_odeiv2_driver_apply(odedrive, &t, ti, y);

            if (status != GSL_SUCCESS)
            {
                DrawText(TextFormat("error, gsl return value=%d ", status), SCREEN_WIDTH / 2, (SCREEN_HEIGHT / 2) + 100, 20, RED);
                pause = !pause;
            } 
            int screenXr1 = t + SCREEN_WIDTH / 2;
            int screenYr1 = y[0] + SCREEN_WIDTH /2;
            DrawPixel(screenXr1, screenYr1, GREEN );

        }

        EndDrawing();

    }

    CloseWindow(); 
/*
    gsl_matrix_free(A);
    gsl_matrix_free(B);
    gsl_permutation_free(p);
*/
    gsl_odeiv2_driver_free(odedrive);
    return 0;

}