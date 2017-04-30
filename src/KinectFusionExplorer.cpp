//------------------------------------------------------------------------------
// <copyright file="KinectFusionExplorer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// System includes
#include "stdafx.h"
#include <iostream>

// Project includes
#include "resource.h"
#include "KinectFusionExplorer.h"
#include "KinectFusionProcessorFrame.h"
#include "KinectFusionHelper.h"
#include "tinyxml2.h"

#define MIN_DEPTH_DISTANCE_MM 500   // Must be greater than 0
#define MAX_DEPTH_DISTANCE_MM 8000
#define MIN_INTEGRATION_WEIGHT 1    // Must be greater than 0
#define MAX_INTEGRATION_WEIGHT 1000

#define WM_FRAMEREADY           (WM_USER + 0)
#define WM_UPDATESENSORSTATUS   (WM_USER + 1)

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>


int main(int nCmdShow)
{
	tinyxml2::XMLDocument doc;
	doc.LoadFile( "utf8test.xml" );
	tinyxml2::XMLElement* root = doc.FirstChildElement();
	tinyxml2::XMLElement* fChild = root -> FirstChildElement();
	const char* name = fChild->Attribute("name");
	std::cout << name << std::endl;
	const char* title = fChild -> GetText();
	std::cout << title << std::endl;
	std::cout << doc.FirstChildElement() << std::endl;

	CKinectFusionExplorer application;
	application.Run(nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
CKinectFusionExplorer::CKinectFusionExplorer() :
m_hWnd(nullptr),
    m_pD2DFactory(nullptr),
    m_pDrawReconstruction(nullptr),
    m_pDrawTrackingResiduals(nullptr),
    m_pDrawDepth(nullptr),
    m_bSavingMesh(false),
    m_saveMeshFormat(Stl),
    m_bInitializeError(false),
    m_bColorCaptured(false),
    m_bUIUpdated(false)
{
}

/// <summary>
/// Destructor
/// </summary>
CKinectFusionExplorer::~CKinectFusionExplorer()
{
    // clean up Direct2D renderer
    SAFE_DELETE(m_pDrawReconstruction);

    // clean up Direct2D renderer
    SAFE_DELETE(m_pDrawTrackingResiduals);

    // clean up Direct2D renderer
    SAFE_DELETE(m_pDrawDepth);

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CKinectFusionExplorer::Run(int nCmdShow)
{
    MSG       msg = {0};
	
	if (FAILED(m_processor.SetParams(m_params)))
	{
		m_bInitializeError = true;
	}
	m_saveMeshFormat = m_params.m_saveMeshType;

    // Main message loop
    while (WM_QUIT != msg.message)
    {
		m_processor.Update();
		std::cout << m_processor.GetElapsedTime() << std::endl;
		if (m_processor.GetElapsedTime() > 10000)
		{
			std::cout << "Saving mesh" << std::endl;
			SaveMesh();
			break;

		}

    }

    return static_cast<int>(msg.wParam);
}



HRESULT CKinectFusionExplorer::SaveMesh()
{
        m_bSavingMesh = true;

        // Pause integration while we're saving
        bool wasPaused = m_params.m_bPauseIntegration;
        m_params.m_bPauseIntegration = true;
        m_processor.SetParams(m_params);

        INuiFusionColorMesh *mesh = nullptr;
        HRESULT hr = m_processor.CalculateMesh(&mesh);

        if (SUCCEEDED(hr))
        {
            // Save mesh
            hr = SaveMeshFile(mesh, m_saveMeshFormat);

            if (SUCCEEDED(hr))
            {
                std::cout << "Saved Kinect Fusion mesh." << std::endl;
            }
            else if (HRESULT_FROM_WIN32(ERROR_CANCELLED) == hr)
            {
                std::cout << "Mesh save canceled." << std::endl;
            }
            else
            {
                std::cout << "Error saving Kinect Fusion mesh!" << std::endl;
            }

            // Release the mesh
            SafeRelease(mesh);
        }
        else
        {
            std::cout << "Failed to create mesh of reconstruction." << std::endl;
        }

        // Restore pause state of integration
        m_params.m_bPauseIntegration = wasPaused;
        m_processor.SetParams(m_params);

        m_bSavingMesh = false;
		return hr;
}

/// <summary>
/// Handle a completed frame from the Kinect Fusion processor.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
void CKinectFusionExplorer::HandleCompletedFrame()
{
    KinectFusionProcessorFrame const* pFrame = nullptr;

    // Flush any extra WM_FRAMEREADY messages from the queue
    MSG msg;
    while (PeekMessage(&msg, m_hWnd, WM_FRAMEREADY, WM_FRAMEREADY, PM_REMOVE)) {}

    m_processor.LockFrame(&pFrame);

    if (!m_bSavingMesh) // don't render while a mesh is being saved
    {
        if (m_processor.IsVolumeInitialized())
        {
            m_pDrawDepth->Draw(pFrame->m_pDepthRGBX, pFrame->m_cbImageSize);
            m_pDrawReconstruction->Draw(pFrame->m_pReconstructionRGBX, pFrame->m_cbImageSize);
            m_pDrawTrackingResiduals->Draw(pFrame->m_pTrackingDataRGBX, pFrame->m_cbImageSize);
        }

        SetStatusMessage(pFrame->m_statusMessage);
        SetFramesPerSecond(pFrame->m_fFramesPerSecond);
    }

    if (!m_bUIUpdated && m_processor.IsVolumeInitialized())
    {
        const int Mebi = 1024 * 1024;

        // We now create both a color and depth volume, doubling the required memory, so we restrict
        // which resolution settings the user can choose when the graphics card is limited in memory.
        if (pFrame->m_deviceMemory <= 1 * Mebi)  // 1GB
        {
            // Disable 640 voxel resolution in all axes - cards with only 1GB cannot handle this
            HWND hButton = GetDlgItem(m_hWnd, IDC_VOXELS_X_640);
            EnableWindow(hButton, FALSE);
            hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Y_640);
            EnableWindow(hButton, FALSE);
            hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Z_640);
            EnableWindow(hButton, FALSE);

            if (Is64BitApp() == FALSE)
            {
                // Also disable 512 voxel resolution in one arbitrary axis on 32bit machines
                hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Y_512);
                EnableWindow(hButton, FALSE);
            }
        }
        else if (pFrame->m_deviceMemory <= 2 * Mebi)  // 2GB
        {
            if (Is64BitApp() == FALSE)
            {
                // Disable 640 voxel resolution in one arbitrary axis on 32bit machines
                HWND hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Y_640);
                EnableWindow(hButton, FALSE);
            }
            // True 64 bit apps seem to be more able to cope with large volume sizes.
        }

        m_bUIUpdated = true;
    }

    m_bColorCaptured = pFrame->m_bColorCaptured;

    m_processor.UnlockFrame();
}

/// <summary>
/// Save Mesh to disk.
/// </summary>
/// <param name="mesh">The mesh to save.</param>
/// <returns>indicates success or failure</returns>
HRESULT CKinectFusionExplorer::SaveMeshFile(INuiFusionColorMesh* pMesh, KinectFusionMeshTypes saveMeshType)
{
    HRESULT hr = S_OK;

    if (nullptr == pMesh)
    {
        return E_INVALIDARG;
    }

    CComPtr<IFileSaveDialog> pSaveDlg;

    // Create the file save dialog object.
    hr = pSaveDlg.CoCreateInstance(__uuidof(FileSaveDialog));

    if (FAILED(hr))
    {
        return hr;
    }

    // Set the dialog title
    hr = pSaveDlg->SetTitle(L"Save Kinect Fusion Mesh");
    if (SUCCEEDED(hr))
    {
        // Set the button text
        hr = pSaveDlg->SetOkButtonLabel (L"Save");
        if (SUCCEEDED(hr))
        {
            // Set a default filename
            if (Stl == saveMeshType)
            {
                hr = pSaveDlg->SetFileName(L"MeshedReconstruction.stl");
            }
            else if (Obj == saveMeshType)
            {
                hr = pSaveDlg->SetFileName(L"MeshedReconstruction.obj");
            }
            else if (Ply == saveMeshType)
            {
                hr = pSaveDlg->SetFileName(L"MeshedReconstruction.ply");
            }

            if (SUCCEEDED(hr))
            {
                // Set the file type extension
                if (Stl == saveMeshType)
                {
                    hr = pSaveDlg->SetDefaultExtension(L"stl");
                }
                else if (Obj == saveMeshType)
                {
                    hr = pSaveDlg->SetDefaultExtension(L"obj");
                }
                else if (Ply == saveMeshType)
                {
                    hr = pSaveDlg->SetDefaultExtension(L"ply");
                }

                if (SUCCEEDED(hr))
                {
                    // Set the file type filters
                    if (Stl == saveMeshType)
                    {
                        COMDLG_FILTERSPEC allPossibleFileTypes[] = {
                            { L"Stl mesh files", L"*.stl" },
                            { L"All files", L"*.*" }
                        };

                        hr = pSaveDlg->SetFileTypes(
                            ARRAYSIZE(allPossibleFileTypes),
                            allPossibleFileTypes);
                    }
                    else if (Obj == saveMeshType)
                    {
                        COMDLG_FILTERSPEC allPossibleFileTypes[] = {
                            { L"Obj mesh files", L"*.obj" },
                            { L"All files", L"*.*" }
                        };

                        hr = pSaveDlg->SetFileTypes(
                            ARRAYSIZE(allPossibleFileTypes),
                            allPossibleFileTypes );
                    }
                    else if (Ply == saveMeshType)
                    {
                        COMDLG_FILTERSPEC allPossibleFileTypes[] = {
                            { L"Ply mesh files", L"*.ply" },
                            { L"All files", L"*.*" }
                        };

                        hr = pSaveDlg->SetFileTypes(
                            ARRAYSIZE(allPossibleFileTypes),
                            allPossibleFileTypes );
                    }

                    if (SUCCEEDED(hr))
                    {
                        // Show the file selection box
                        hr = pSaveDlg->Show(m_hWnd);

                        // Save the mesh to the chosen file.
                        if (SUCCEEDED(hr))
                        {
                            CComPtr<IShellItem> pItem;
                            hr = pSaveDlg->GetResult(&pItem);

                            if (SUCCEEDED(hr))
                            {
                                LPOLESTR pwsz = nullptr;
                                hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pwsz);

                                if (SUCCEEDED(hr))
                                {
                                    SetStatusMessage(L"Saving mesh file, please wait...");
                                    SetCursor(LoadCursor(nullptr, MAKEINTRESOURCE(IDC_WAIT)));

                                    if (Stl == saveMeshType)
                                    {
                                        hr = WriteBinarySTLMeshFile(pMesh, pwsz);
                                    }
                                    else if (Obj == saveMeshType)
                                    {
                                        hr = WriteAsciiObjMeshFile(pMesh, pwsz);
                                    }
                                    else if (Ply == saveMeshType)
                                    {
                                        hr = WriteAsciiPlyMeshFile(pMesh, pwsz, true, m_bColorCaptured);
                                    }

                                    CoTaskMemFree(pwsz);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return hr;
}

///////////////////////////////////////////////////////////////////////////////////////////



/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
void CKinectFusionExplorer::SetStatusMessage(const WCHAR * szMessage)
{
    size_t length = 0;
    if (FAILED(StringCchLength(
        szMessage,
        KinectFusionProcessorFrame::StatusMessageMaxLen,
        &length)))
    {
        length = 0;
    }

    if (length > 0)
    {
        SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
        m_tickLastStatus = GetTickCount64();
    }
    else
    {
        // Clear the status message after a timeout (as long as frames are flowing)
        if (GetTickCount64() - m_tickLastStatus > cStatusTimeoutInMilliseconds &&
            m_fFramesPerSecond > 0)
        {
            SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, 0);
            m_tickLastStatus = GetTickCount64();
        }
    }
}

/// <summary>
/// Set the frames-per-second message
/// </summary>
/// <param name="fFramesPerSecond">current frame rate</param>
void CKinectFusionExplorer::SetFramesPerSecond(float fFramesPerSecond)
{
    if (fFramesPerSecond != m_fFramesPerSecond)
    {
        m_fFramesPerSecond = fFramesPerSecond;
        WCHAR str[MAX_PATH] = {0};
        if (fFramesPerSecond > 0)
        {
            swprintf_s(str, ARRAYSIZE(str), L"%5.2f FPS", fFramesPerSecond);
        }

        SendDlgItemMessageW(m_hWnd, IDC_FRAMES_PER_SECOND, WM_SETTEXT, 0, (LPARAM)str);
    }
}
