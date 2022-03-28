#include "landscape2_x_data.h"
#include "tok_sample_glapp.h"

#define TOKAMAK_SAMPLE_NAME "sphere cylinder collision"
const char* tokamakSampleTitle = TOKAMAK_SAMPLE_TITLE_COMMON TOKAMAK_SAMPLE_NAME;

neV4 vLightWorld[NUM_LIGHT] = { { 1.f, 2.f, 1.f, 0.f }, { -1.f, 1.f, 1.f, 0.f } };
neV4 vLightColor[NUM_LIGHT] = { { 0.7f, 0.7f, 0.7f, 0.f }, { 0.5f, 0.5f, 0.5f, 0.f } };

// const s32 MAX_OVERLAPPED_PAIR = 300;
const s32 WALL_NUMBER = 1;
// const f32 EPSILON = 0.1f;

struct DemoData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
};

DemoData gFloor = { { 0.0f, -11.0f, 0.0f }, { 200.0f, 2.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } };

void BreakageCallbackFn(neByte* originalBody, neBodyType bodyType, neGeometry* brokenGeometry, neRigidBody* newBody);

class CSampleSCCollision
{
public:
    CSampleSCCollision()
    {
        paused = false;
    }

    void Initialise();

    void Shutdown();

    void Process();

    void InititialisePhysics();

    void InititialiseRenderPrimitives();

    void MakeParticle(neV3 position, s32 index);

    void MakeTree(neV3 position, s32 index);

    void DisplayAnimatedBodies(neAnimatedBody** ab, s32 count);

    void DisplayRigidBodies(neRigidBody** rb, s32 count);

public:
    enum
    {
        NUMBER_OF_PARTICLES = 100,

        NUMBER_OF_TREES = 5,

        BRANCH_PER_TREE = 4,

        GEOMETRY_PER_TREE = BRANCH_PER_TREE + 2,

        NUMBER_OF_BODIES = NUMBER_OF_PARTICLES + NUMBER_OF_TREES * BRANCH_PER_TREE,

        NUMBER_OF_TOKENS = NUMBER_OF_PARTICLES + NUMBER_OF_TREES * GEOMETRY_PER_TREE,
    };

    neSimulator* sim;

    neRigidBody* particles[NUMBER_OF_PARTICLES];

    neRigidBody* branches[NUMBER_OF_TREES * BRANCH_PER_TREE];

    neAnimatedBody* trees[NUMBER_OF_TREES];

    CRenderPrimitive renderPrimitives[NUMBER_OF_TOKENS];

    neAllocatorDefault all;

    nePerformanceReport perfReport;

    bool paused;

    CRenderPrimitive groundRender;
    neAnimatedBody* ground;

    CRenderPrimitive terrainRender;

    s32 nextBullet;

    s32 nextFreeRenderPrimitive;

    s32 nextFreeBranch;
};

CSampleSCCollision sample;

void CSampleSCCollision::Initialise()
{
    InititialisePhysics();
    InititialiseRenderPrimitives();
    nextBullet = 0;
    nextFreeBranch = 0;
    nextFreeRenderPrimitive = 0;
}

void CSampleSCCollision::Process()
{
    static s32 nextBullet = 0;
    if (sdlGetAsyncKeyStateOnce(SDLK_t))
    {
        neV3 pos;
        neV3 dir;
        neV3 eyept = g_Camera.GetEyePt();
        neV3 lookatpt = g_Camera.GetLookAtPt();
        neV3 d3ddir;
        d3ddir = lookatpt - eyept;

        pos.Set(eyept);
        dir.Set(d3ddir);
        dir.Normalize();

        pos = pos + dir * 10.0f;

        particles[nextBullet]->SetPos(pos);
        particles[nextBullet]->SetVelocity(dir * 30.0f);

        nextBullet = (nextBullet + 1) % NUMBER_OF_PARTICLES;
    }
}

void CSampleSCCollision::Shutdown()
{
    sim->FreeTerrainMesh();
    neSimulator::DestroySimulator(sim);
    sim = NULL;
}

void CSampleSCCollision::MakeParticle(neV3 position, s32 index)
{
    // const f32 groundLevel = -10.0f;

    s32 cur;

    cur = index;

    particles[cur] = sim->CreateRigidParticle();

    particles[cur]->GravityEnable(false);

    neGeometry* geom = particles[cur]->AddGeometry();

    geom->SetSphereDiameter(0.8f);

    particles[cur]->UpdateBoundingInfo();

    f32 mass = 1.0f;

    particles[cur]->SetInertiaTensor(neSphereInertiaTensor(0.8f, mass));

    particles[cur]->SetMass(mass);

    particles[cur]->SetPos(position);
}

struct TreeData
{
    f32 diameter;
    f32 height;
    neV3 pos;
    neV3 rotation;
    neV3 colour;
    neGeometry::neBreakFlag breakFlag;
    f32 breakageMass;
    f32 breakageMagnitude;
    f32 breakageAbsorption;
};

TreeData treeData[CSampleSCCollision::GEOMETRY_PER_TREE] =
{
    { 1.5f, 10.0f, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.3f, 0.1f }, neGeometry::NE_BREAK_DISABLE, 2.0f, 0.0f, 0.0f },
    { 1.2f, 10.0f, { 0.0f, 10.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.3f, 0.1f }, neGeometry::NE_BREAK_DISABLE, 2.0f, 0.0f, 0.0f },

    { 0.8f, 7.0f, { -4.0f, 3.0f, 0.0f }, { 0.0f, 0.0f, -NE_PI / 4.0f }, { 0.4f, 0.5f, 0.1f }, neGeometry::NE_BREAK_NORMAL, 5.0f, 1.0f, 0.3f },
    { 0.8f, 7.0f, { 4.0f, 6.0f, 0.0f }, { 0.0f, 0.0f, NE_PI / 4.0f }, { 0.4f, 0.5f, 0.1f }, neGeometry::NE_BREAK_NORMAL, 5.0f, 1.0f, 0.3f },

    { 0.8f, 6.0f, { -3.5f, 10.0f, 0.0f }, { 0.0f, 0.0f, -NE_PI / 4.0f }, { 0.4f, 0.5f, 0.1f }, neGeometry::NE_BREAK_NORMAL, 5.0f, 2.0f, 0.3f },
    { 0.8f, 6.0f, { 3.5f, 12.0f, 0.0f }, { 0.0f, 0.0f, NE_PI / 4.0f }, { 0.4f, 0.5f, 0.1f }, neGeometry::NE_BREAK_NORMAL, 5.0f, 2.0f, 0.3f },

};

void CSampleSCCollision::MakeTree(neV3 position, s32 index)
{
    // const f32 groundLevel = -10.0f;

    s32 cur;

    cur = index;

    trees[cur] = sim->CreateAnimatedBody();

    trees[cur]->SetPos(position);

    for (s32 i = 0; i < GEOMETRY_PER_TREE; i++)
    {
        neGeometry* geom = trees[cur]->AddGeometry();

        geom->SetCylinder(treeData[i].diameter, treeData[i].height);

        neT3 trans;

        trans.SetIdentity();

        trans.pos = treeData[i].pos;

        trans.rot.RotateXYZ(treeData[i].rotation);

        geom->SetTransform(trans);

        geom->SetBreakageFlag(treeData[i].breakFlag);

        geom->SetBreakageMass(treeData[i].breakageMass);

        geom->SetBreakageInertiaTensor(neCylinderInertiaTensor(treeData[i].height, treeData[i].height, treeData[i].breakageMass));

        geom->SetBreakageMagnitude(treeData[i].breakageMagnitude);

        geom->SetBreakageAbsorption(treeData[i].breakageAbsorption);

        neV3 plane;

        plane.Set(0.0f, 1.0f, 0.0f);

        geom->SetBreakagePlane(plane);
    }
    trees[cur]->UpdateBoundingInfo();
}

void BreakageCallbackFn(neByte* originalBody, neBodyType bodyType, neGeometry* brokenGeometry, neRigidBody* newBody)
{
    (void)originalBody;
    (void)bodyType;
    (void)brokenGeometry;
    sample.branches[sample.nextFreeBranch] = (neRigidBody*)newBody;

    sample.nextFreeBranch++;
}

neV3 treePosition[CSampleSCCollision::NUMBER_OF_TREES] =
{
    { 0.0f, 5.0f, 0.0f },
    { 25.0f, 5.0f, -20.0f },
    { -20.0f, 0.0f, 30.0f },
    { 30.0f, 5.0f, 10.0f },
    { -30.0f, 0.0f, -20.0f },
};

void CSampleSCCollision::InititialiseRenderPrimitives()
{
    s32 curToken = 0;

    for (s32 i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        particles[i]->BeginIterateGeometry();

        neGeometry* geom = particles[i]->GetNextGeometry();

        while (geom)
        {

            f32 diameter;
            geom->GetSphereDiameter(diameter);
            renderPrimitives[curToken].SetGraphicSphere(diameter / 2.0f);

            geom->SetUserData(&renderPrimitives[curToken]);

            if ((curToken % 4) == 0)
            {
                renderPrimitives[curToken].SetDiffuseColor(0.8f, 0.2f, 0.2f, 1.0f);
            }
            else if ((curToken % 4) == 1)
            {
                renderPrimitives[curToken].SetDiffuseColor(0.2f, 0.4f, 0.6f, 1.0f);
            }
            else if ((curToken % 4) == 2)
            {
                renderPrimitives[curToken].SetDiffuseColor(0.2f, 0.6f, 0.2f, 1.0f);
            }
            else
            {
                renderPrimitives[curToken].SetDiffuseColor(0.6f, 0.6f, 0.2f, 1.0f);
            }

            curToken++;
            geom = particles[i]->GetNextGeometry();
        }
    }

    for (int i = 0; i < NUMBER_OF_TREES; i++)
    {
        trees[i]->BeginIterateGeometry();

        neGeometry* geom = trees[i]->GetNextGeometry();
        s32 b = 0;
        while (geom)
        {
            {
                neV3 scale;
                if (geom->GetBoxSize(scale))
                {
                    renderPrimitives[curToken].SetGraphicBox(scale.X(), scale.Y(), scale.Z());
                }
                else
                {
                    f32 height, diameter;

                    if (geom->GetCylinder(diameter, height))
                    {
                        renderPrimitives[curToken].SetGraphicCylinder(diameter / 2.0f, height);
                    }
                    else if (geom->GetSphereDiameter(diameter))
                    {
                        renderPrimitives[curToken].SetGraphicSphere(diameter / 2.0);
                    }
                }

                geom->SetUserData(&renderPrimitives[curToken]);

                if (geom->GetBreakageFlag() == neGeometry::NE_BREAK_DISABLE)
                {
                    renderPrimitives[curToken].SetDiffuseColor(treeData[b].colour[0],
                            treeData[b].colour[1],
                            treeData[b].colour[2],
                            1.0f);
                }
                else if (geom->GetBreakageFlag() == neGeometry::NE_BREAK_NORMAL)
                {
                    renderPrimitives[curToken].SetDiffuseColor(treeData[b].colour[0],
                            treeData[b].colour[1],
                            treeData[b].colour[2],
                            1.0f);
                }
            }
            curToken++;
            geom = trees[i]->GetNextGeometry();

            b++;
        }
    }
    nextFreeRenderPrimitive = curToken;
}

void CSampleSCCollision::DisplayRigidBodies(neRigidBody** rb, s32 count /* , IDirect3DDevice9* pd3dDevice*/)
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

void CSampleSCCollision::DisplayAnimatedBodies(neAnimatedBody** ab, s32 count /*, IDirect3DDevice9* pd3dDevice*/)
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

                t->Render(&worldTrans);
            }
            geom = body->GetNextGeometry();
        }
        ab++;
    }
}
void CSampleSCCollision::InititialisePhysics()
{
    for (s32 i = 0; i < NUMBER_OF_TREES * BRANCH_PER_TREE; i++)
    {
        branches[i] = NULL;
    }
    neV3 gravity;
    gravity.Set(0.0f, -10.0f, 0.0f);

    neSimulatorSizeInfo sizeInfo;

    sizeInfo.rigidBodiesCount = NUMBER_OF_TREES * BRANCH_PER_TREE;
    sizeInfo.rigidParticleCount = NUMBER_OF_PARTICLES;
    sizeInfo.animatedBodiesCount = WALL_NUMBER + NUMBER_OF_TREES;
    sizeInfo.geometriesCount = NUMBER_OF_TOKENS + WALL_NUMBER;

    {
        // dont need any of these

        sizeInfo.constraintsCount = 0;
    }

    s32 totalBody = sizeInfo.rigidBodiesCount + sizeInfo.animatedBodiesCount;

    sizeInfo.overlappedPairsCount = totalBody * (totalBody - 1) / 2;

    sim = neSimulator::CreateSimulator(sizeInfo, &all, &gravity);

    sim->SetBreakageCallback(BreakageCallbackFn);

    neV3 position;

    position.SetZero();

    for (s32 j = 0; j < NUMBER_OF_PARTICLES; j++)
    {
        position.Set(2.0f * j, 2.0f, 100.0f);

        MakeParticle(position, j);
    }
    for (s32 j = 0; j < NUMBER_OF_TREES; j++)
    {
        MakeTree(treePosition[j], j);
    }

    // SetUpTerrain

    terrainRender.SetGraphicMesh(terVert, sizeof(terVert) / sizeof(float) / 3, terNorm, sizeof(terNorm) / sizeof(float) / 3, terQuad, sizeof(terQuad) / sizeof(uint16_t), 4);
    terrainRender.SetDiffuseColor(0.1f, 0.5f, 0.1f, 1.0f);

    neTriangleMesh triMesh;

    triMesh.vertexCount = terrainRender.mVerts.size() / 3;

    triMesh.triangleCount = terrainRender.mTris.size() / 3;

    neV3* verts = new neV3[triMesh.vertexCount];

    //
    for (int i = 0; i != triMesh.vertexCount; ++i)
    {
        verts[i].Set(terrainRender.mVerts[i * 3], terrainRender.mVerts[i * 3 + 1], terrainRender.mVerts[i * 3 + 2]);
    }

    triMesh.vertices = verts;

    neTriangle* tri = new neTriangle[triMesh.triangleCount];

    s32* triindex = new s32[triMesh.triangleCount * 3];

    for (int i = 0; i != triMesh.triangleCount * 3; ++i)
    {
        triindex[i] = terrainRender.mTris[i];
    }

    //

    //

    for (s32 i = 0; i < triMesh.triangleCount; i++)
    {
        tri[i].indices[0] = triindex[i * 3];
        tri[i].indices[1] = triindex[i * 3 + 1];
        tri[i].indices[2] = triindex[i * 3 + 2];
        tri[i].materialID = 0;
        tri[i].flag = neTriangle::NE_TRI_TRIANGLE;
        // tri[i].flag = neTriangle::NE_TRI_HEIGHT_MAP;
    }
    triMesh.triangles = tri;

    sim->SetTerrainMesh(&triMesh);

    // SetUpRoom

    ground = sim->CreateAnimatedBody();

    neGeometry* geom = ground->AddGeometry();

    geom->SetBoxSize(gFloor.boxSize);

    ground->UpdateBoundingInfo();

    ground->SetPos(gFloor.pos);

    groundRender.SetGraphicBox(gFloor.boxSize[0], gFloor.boxSize[1], gFloor.boxSize[2]);
}

void MyAppInit()
{
    neV3 vecEye;
    vecEye.Set(-10.0f, 25.0f, 40.0f);
    neV3 vecAt;
    vecAt.Set(0.0f, 10.0f, 1.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    for (s32 i = 0; i < NUM_LIGHT; i++)
    {
        vLightWorld[i].Normalize();
    }
    sample.Initialise();
};

void OnMyAppFrameMove(double fTime, float fElapsedTime)
{
    (void)fTime;
    (void)fElapsedTime;
    ////////////////////////////////////////////////////////
    sample.Process();

    if (!sample.paused)
    {
        sample.sim->Advance(1.0f / 60.0f, 1);
    }
}

void OnMyAppFrameRender()
{
    neT3 terrainTrans;
    terrainTrans.SetIdentity();
    sample.terrainRender.Render(&terrainTrans);

    neT3 t;
    t = sample.ground->GetTransform();
    t.MakeD3DCompatibleMatrix();
    sample.groundRender.Render(&t);

    sample.DisplayRigidBodies(sample.particles, CSampleSCCollision::NUMBER_OF_PARTICLES /*,pd3dDevice*/);
    sample.DisplayRigidBodies(sample.branches, CSampleSCCollision::NUMBER_OF_TREES * CSampleSCCollision::BRANCH_PER_TREE /*,pd3dDevice*/);
    sample.DisplayAnimatedBodies(sample.trees, CSampleSCCollision::NUMBER_OF_TREES /*,pd3dDevice*/);

    const char* str[8];

    str[0] = "Tokamak demo - Sphere and Cylinder collision Demo - (c) 2010 Tokamak Ltd";
    str[1] = "Controls:";
    str[2] = "'P' -> pause/unpause the simulation";
    str[3] = "'T' -> Fire particles";
    str[4] = "100 sphere rigid particles interact with arbitrary landscape mesh.";
    str[5] = "Trees constructed with cylinders and branches are specified";
    str[6] = "to break off when hit.";
    str[7] = "";

    MyRenderText(str, 8);
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
        case SDLK_r:
        {
        } break;

        default:
            break;
        }
    }
}
