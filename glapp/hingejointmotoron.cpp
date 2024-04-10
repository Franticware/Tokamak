#include "tok_sample_glapp.h"

#define TOKAMAK_SAMPLE_NAME "hinge joint motor on"
const char* tokamakSampleTitle = TOKAMAK_SAMPLE_TITLE_COMMON TOKAMAK_SAMPLE_NAME;

neV4 vLightWorld[NUM_LIGHT] = { { 1.f, 2.f, 1.f, 0.f }, { -1.f, 1.f, 1.f, 0.f } };

neV4 vLightColor[NUM_LIGHT] = { { 0.7f, 0.7f, 0.7f, 0.f }, { 0.5f, 0.5f, 0.5f, 0.f } };

// const int32_t MAX_OVERLAPPED_PAIR = 300;
const int32_t WALL_NUMBER = 1;
// const float EPSILON  = 0.1f;

struct DemoData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
};

DemoData gFloor = { { 0.0f, -11.0f, 0.0f }, { 200.0f, 2.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } };

class CSampleHingeJointMotorOn
{
public:
    CSampleHingeJointMotorOn()
    {
        paused = false;
    }

    void Initialise();

    void Shutdown();

    void Process();

    void InititialisePhysics();

    void Reset();

    void DisplayRigidBodies(neRigidBody** rb, int32_t count);

    void DisplayAnimatedBodies(neAnimatedBody** ab, int32_t count);

public:
    enum
    {
        N_BODY = 5,

        N_TOTAL_BODIES = N_BODY,
    };

    neSimulator* sim;

    neRigidBody* rigidBodies[N_TOTAL_BODIES];

    CRenderPrimitive renderPrimitives[N_TOTAL_BODIES];

    neJoint* lastJoint;

    neAllocatorDefault all;

    nePerformanceReport perfReport;

    bool paused;

    CRenderPrimitive groundRender;
    neAnimatedBody* ground;
};

CSampleHingeJointMotorOn sample;

void CSampleHingeJointMotorOn::DisplayRigidBodies(neRigidBody** rb, int32_t count)
{
    while (count-- && *rb)
    {
        neRigidBody* body = *rb;

        neT3 trans = body->GetTransform();

        body->BeginIterateGeometry();

        neGeometry* geom = body->GetNextGeometry();

        while (geom)
        {
            CRenderPrimitive* t = (CRenderPrimitive*)geom->GetUserData().p;

            if (t)
            {
                neT3 geomTrans = geom->GetTransform();

                neT3 worldTrans = trans * geomTrans;

                worldTrans.MakeD3DCompatibleMatrix();

                t->Render(&worldTrans);
            }
            geom = body->GetNextGeometry();
        }
        rb++;
    }
}

void CSampleHingeJointMotorOn::DisplayAnimatedBodies(neAnimatedBody** ab, int32_t count)
{
    while (count-- && *ab)
    {
        neAnimatedBody* body = *ab;

        neT3 trans = body->GetTransform();

        body->BeginIterateGeometry();

        neGeometry* geom = body->GetNextGeometry();

        while (geom)
        {
            CRenderPrimitive* t = (CRenderPrimitive*)geom->GetUserData().p;

            if (t)
            {
                neT3 geomTrans = geom->GetTransform();

                neT3 worldTrans = trans * geomTrans;

                worldTrans.MakeD3DCompatibleMatrix();

                t->Render(/*pd3dDevice,*/ &worldTrans);
            }
            geom = body->GetNextGeometry();
        }
        ab++;
    }
}
void CSampleHingeJointMotorOn::Initialise()
{
    InititialisePhysics();
}

void CSampleHingeJointMotorOn::Process()
{
    if (sdlGetAsyncKeyState(SDLK_r))
    {
        Reset();
        return;
    }

    int32_t actOnBody = 0; // N_TOTAL_BODIES - 1;// -20;

    if (sdlGetAsyncKeyState(SDLK_t))
    {
        neV3 vel;

        vel.Set(0.0f, 30.0f, 0.0f);

        rigidBodies[actOnBody]->SetVelocity(vel);
    }
}

void CSampleHingeJointMotorOn::Shutdown()
{
    neSimulator::DestroySimulator(sim);

    sim = NULL;
}

void CSampleHingeJointMotorOn::Reset()
{
    Shutdown();

    InititialisePhysics();
}

void CSampleHingeJointMotorOn::InititialisePhysics()
{
    float ypos = -7;

    neV3 gravity;
    gravity.Set(0.0f, -9.0f, 0.0f);
    // neV3 gravity; gravity.Set(-9.0f, 0.0f, 0.0f);

    // float linkLength = 2.2f;
    // float linkLength = -0.0f;

    neSimulatorSizeInfo sizeInfo;

    sizeInfo.rigidBodiesCount = N_TOTAL_BODIES;
    sizeInfo.animatedBodiesCount = WALL_NUMBER;
    sizeInfo.geometriesCount = N_TOTAL_BODIES + WALL_NUMBER;
    sizeInfo.overlappedPairsCount = 2000; // MAX_OVERLAPPED_PAIR;

    {
        // dont need any of these
        sizeInfo.terrainNodesStartCount = 0;
        sizeInfo.rigidParticleCount = 0;
    }

    sim = neSimulator::CreateSimulator(sizeInfo, &all, &gravity);

    // create the kart body

    float mass = 1.0f;

    rigidBodies[0] = sim->CreateRigidBody();

    rigidBodies[0]->SetMass(mass);

    neGeometry* geom = rigidBodies[0]->AddGeometry();

    neV3 boxSize;
    boxSize.Set(3.0f, 3.f, 1.0f);

    geom->SetBoxSize(boxSize);

    rigidBodies[0]->UpdateBoundingInfo();

    rigidBodies[0]->SetInertiaTensor(neBoxInertiaTensor(boxSize, mass));

    neV3 pos;
    pos.Set(0.0f, ypos, 0.0f);

    rigidBodies[0]->SetPos(pos);

    rigidBodies[0]->SetSleepingParameter(.0f);

    // create the 4 wheels (sphere)

    int32_t j = 1;

    ypos -= 2.0f;

    for (int i = 0; i < 4; i++)
    {
        neRigidBody* rb;

        rigidBodies[j++] = rb = sim->CreateRigidBody();

        rb->SetMass(mass);

        geom = rb->AddGeometry();

        geom->SetBoxSize(2.0f, .5f, 1.0f); // SetSphereDiameter(1.0f);

        // geom->SetSphereDiameter(1.0f);

        rb->UpdateBoundingInfo();

        // rb->SetInertiaTensor(neSphereInertiaTensor(1.0f, 0.1f));

        rb->SetInertiaTensor(neBoxInertiaTensor(2.0f, .5f, 1.0f, mass));

        neV3 pos;

        switch (i)
        {
        case 0:
            pos.Set(-1.5f, ypos, 1.0f);
            break;
        case 1:
            pos.Set(-1.5f, ypos, -1.0f);
            break;
        case 2:
            pos.Set(1.5f, ypos, 1.0f);
            break;
        case 3:
            pos.Set(1.5f, ypos, -1.0f);
            break;
        }
        rb->SetPos(pos);

        neJoint* joint = sim->CreateJoint(rigidBodies[0], rb);

        joint->SetIteration(1);

        neT3 jointFrame;

        jointFrame.rot[0].Set(1.0f, 0.0f, 0.0f);
        jointFrame.rot[1].Set(0.0f, 0.0f, -1.0f);
        jointFrame.rot[2].Set(0.0f, 1.0f, 0.0f);
        jointFrame.pos = pos;

        joint->SetJointFrameWorld(jointFrame);
        joint->SetType(neJoint::NE_JOINT_HINGE);
        joint->Enable(true);
        joint->EnableMotor(true);
        joint->SetMotor(neJoint::NE_MOTOR_SPEED, 1.f, 50.f);
    }

    for (int i = 0; i < N_BODY; i++)
    {
        rigidBodies[i]->BeginIterateGeometry();

        neGeometry* geom = rigidBodies[i]->GetNextGeometry();

        neV3 scale;
        if (geom->GetBoxSize(scale))
        {
            renderPrimitives[i].SetGraphicBox(scale.X(), scale.Y(), scale.Z());
        }
        else
        {
            float height, diameter;

            if (geom->GetCylinder(diameter, height))
            {
                renderPrimitives[i].SetGraphicCylinder(diameter / 2.0f, height);
            }
            else if (geom->GetSphereDiameter(diameter))
            {
                renderPrimitives[i].SetGraphicSphere(diameter / 2.0);
            }
        }

        geom->SetUserData(&renderPrimitives[i]);
    }

    // SetUpRoom
    {

        ground = sim->CreateAnimatedBody();

        neGeometry* geom = ground->AddGeometry();

        geom->SetBoxSize(gFloor.boxSize);

        ground->UpdateBoundingInfo();

        ground->SetPos(gFloor.pos);

        groundRender.SetGraphicBox(gFloor.boxSize[0], gFloor.boxSize[1], gFloor.boxSize[2]);
    }
}

void MyAppInit()
{
    neV3 vecEye;
    vecEye.Set(-10.0f, 5.0f, 40.0f);
    neV3 vecAt;
    vecAt.Set(0.0f, 0.0f, 1.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    for (int32_t i = 0; i < NUM_LIGHT; i++)
    {
        vLightWorld[i].Normalize();
    }
    sample.Initialise();
};

void OnMyAppFrameMove(double fTime, float fElapsedTime)
{
    (void)fTime;
    (void)fElapsedTime;
    sample.Process();

    if (!sample.paused)
    {
        sample.sim->Advance(1.0f / 60.0f, 1);
    }
}

void OnMyAppFrameRender()
{
    neT3 t;
    t = sample.ground->GetTransform();
    t.MakeD3DCompatibleMatrix();
    sample.groundRender.Render(&t);

    // Display the boxes

    sample.DisplayRigidBodies(sample.rigidBodies, CSampleHingeJointMotorOn::N_TOTAL_BODIES);

    const char* str[7];

    str[0] = "Tokamak demo - Hinge joint with Motor turned on - (c) 2010 Tokamak Ltd";
    str[1] = "Controls:";
    str[2] = "'P' -> pause/unpause the simulation";
    str[3] = "'T' -> lift";
    str[4] = "'R' -> Reset";
    str[5] = "This sample demonstrate the hinge joint motor.";
    str[6] = "";

    MyRenderText(str, 7);
}

void OnMyAppDestroyDevice()
{
    sample.Shutdown();
}

void MyAppKeyboardProc(SDL_Keycode nChar, bool bKeyDown, bool bAltDown)
{
    (void)bAltDown;
    if (bKeyDown)
    {
        switch (nChar)
        {
        case SDLK_p:
        {
            sample.paused = !sample.paused;
        }
        break;

        default:
            break;
        }
    }
}
