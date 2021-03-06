#include "CD3D11Shader.h"
#include "CD3D11Driver.h"
#include "CD3D11HardwareBuffer.h"
#include "os.h"
#include <iostream>
#include <fstream>

#include <D3DCompiler.h>

#define Dx11ShaderDebug 0

using namespace irr;
using namespace irr::video;

video::E_SHADER_VARIABLE_TYPE getShaderVariableTypeId(D3D_SHADER_VARIABLE_TYPE hlslangType, D3D_SHADER_VARIABLE_CLASS hlslclass)
{
    if (hlslclass == D3D_SHADER_VARIABLE_CLASS::D3D_SVC_STRUCT)
        return video::E_SHADER_VARIABLE_TYPE::ESVT_STRUCT;

    switch (hlslangType)
    {
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_VOID:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_VOID;
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_BOOL:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_BOOL;
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_DOUBLE:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_DOUBLE;
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_FLOAT:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_FLOAT;
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_MIN16FLOAT:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_FLOAT16;
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_INT:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_INT;
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_UINT8:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_UINT8;
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_UINT:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_UINT;
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_SAMPLER:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_SAMPLER;
        case D3D_SHADER_VARIABLE_TYPE::D3D_SVT_STRUCTURED_BUFFER:
            return video::E_SHADER_VARIABLE_TYPE::ESVT_BUFFER;
    }
    return video::E_SHADER_VARIABLE_TYPE::ESVT_MAX;
}

ID3DBlob* D3D11CompileShader(System::IO::IFileReader* file, const char* entryPoint, const char* shaderModel, ID3DBlob*& errorMessage)
{
    UINT compileFlags = 0;
    //flags |= D3D10_SHADER_ENABLE_BACKWARDS_COMPATIBILITY;
#if	Dx11ShaderDebug
    // These values allow use of PIX and shader debuggers
    compileFlags |= D3DCOMPILE_DEBUG;
    compileFlags |= D3DCOMPILE_SKIP_OPTIMIZATION | D3DCOMPILE_PREFER_FLOW_CONTROL;
    //flags |= D3D10_SHADER_WARNINGS_ARE_ERRORS;
#else
    // These flags allow maximum performance
    compileFlags |= D3DCOMPILE_ENABLE_STRICTNESS;
    compileFlags |= D3DCOMPILE_OPTIMIZATION_LEVEL3 | D3DCOMPILE_PARTIAL_PRECISION;
#endif

    std::string source;
    source.resize(file->Size());
    file->Read((byte*)source.data(), file->Size());
    //if (file->Read((byte*)source.data(), file->Size()) != file->Size())
    //    throw std::runtime_error("shader not found");

    ID3DBlob* ShaderBuffer;

    HRESULT hr = D3DCompile(
        source.c_str(),         // [in] Pointer to the shader in memory. 
        source.size(),          // [in] Size of the shader in memory.  
        file->FileName.c_str(), // [in] Optional. You can use this parameter for strings that specify error messages.
        nullptr,                // [in] Optional. Pointer to a NULL-terminated array of macro definitions. See D3D_SHADER_MACRO. If not used, set this to NULL. 
        nullptr,                // [in] Optional. Pointer to an ID3DInclude Interface interface for handling include files. Setting this to NULL will cause a compile error if a shader contains a #include. 
        entryPoint,             // [in] Name of the shader-entrypoint function where shader execution begins. 
        shaderModel,            // [in] A string that specifies the shader model; can be any profile in shader model 4 or higher. 
        compileFlags,           // [in] Effect compile flags - no D3DCOMPILE_ENABLE_BACKWARDS_COMPATIBILITY at the first try...
        NULL,                   // [in] Effect compile flags
        &ShaderBuffer,          // [out] A pointer to an ID3DBlob Interface which contains the compiled shader, as well as any embedded debug and symbol-table information. 
        &errorMessage           // [out] A pointer to an ID3DBlob Interface which contains a listing of errors and warnings that occurred during compilation. These errors and warnings are identical to the the debug output from a debugger.
    );

    if (FAILED(hr))
    {
        if (errorMessage)
        {
            LPVOID pCompilErrors = errorMessage->GetBufferPointer();
            printf("[HLSL SHADER] Vertex Shader Compile Error: %s \r\n", (const char*)pCompilErrors);
        }

        os::Printer::log("Error, could not open effect file");
        // Release the vertex shader buffer and pixel shader buffer since they are no longer needed.
        SAFE_RELEASE(ShaderBuffer);
        SAFE_RELEASE(errorMessage);
        throw std::runtime_error("invalid shader");
    }

    return ShaderBuffer;
}

void irr::video::D3D11HLSLProgram::OnDeviceLost(CD3D11Driver * device)
{
}

void irr::video::D3D11HLSLProgram::OnDeviceRestored(CD3D11Driver * device)
{
}

irr::video::D3D11HLSLProgram::D3D11HLSLProgram(video::IVideoDriver * context)
    : CNullShader(context, E_SHADER_LANG::ESV_HLSL_HIGH_LEVEL)
    , D3D11DeviceResource(static_cast<CD3D11Driver*>(context))
    , m_vertexShader(nullptr)
    , m_pixelShader(nullptr)
    , m_hullShader(nullptr)
    , m_domainShader(nullptr)
    , m_computeShader(nullptr)
    , m_geometryShader(nullptr)
    , m_inputLayout(nullptr)
    , m_vertexShaderBytecode(nullptr)
    , pReflectVertexShader(nullptr)
    , pReflectHullShader(nullptr)
    , pReflectDomainShader(nullptr)
    , pReflectComputeShader(nullptr)
    , pReflectGeometryShader(nullptr)
    , pReflectPixelShader(nullptr)
{
}

irr::video::D3D11HLSLProgram::~D3D11HLSLProgram()
{
    SAFE_RELEASE(m_vertexShader)
    SAFE_RELEASE(m_pixelShader)
    SAFE_RELEASE(m_hullShader)
    SAFE_RELEASE(m_domainShader)
    SAFE_RELEASE(m_computeShader)
    SAFE_RELEASE(m_geometryShader)
    SAFE_RELEASE(m_inputLayout)
    SAFE_RELEASE(m_vertexShaderBytecode)
    SAFE_RELEASE(pReflectVertexShader)
    SAFE_RELEASE(pReflectHullShader)
    SAFE_RELEASE(pReflectDomainShader)
    SAFE_RELEASE(pReflectComputeShader)
    SAFE_RELEASE(pReflectGeometryShader)
    SAFE_RELEASE(pReflectPixelShader)

    if (m_inputLayout)
        m_inputLayout->Release();
    m_inputLayout = nullptr;
}

bool irr::video::D3D11HLSLProgram::createVertexShader(ID3D11Device* device, System::IO::IFileReader* file, const char* entryPoint, const char* shaderModel)
{
    SAFE_RELEASE(m_vertexShaderBytecode);

    HRESULT hr = S_OK;
    ID3DBlob* errorMessage;
    m_vertexShaderBytecode = D3D11CompileShader(file, entryPoint, shaderModel, errorMessage);

    // Create the vertex shader from the buffer.
    hr = device->CreateVertexShader(m_vertexShaderBytecode->GetBufferPointer(), m_vertexShaderBytecode->GetBufferSize(), NULL, &m_vertexShader);
    if (FAILED(hr))
    {
        // Release the vertex shader buffer and pixel shader buffer since they are no longer needed.
        SAFE_RELEASE(m_vertexShaderBytecode);
        throw std::runtime_error("invalid shader");
        return false;
    }

    // get description of precompiled shader
    pReflectVertexShader = NULL;
    D3DReflect(
        m_vertexShaderBytecode->GetBufferPointer(),
        m_vertexShaderBytecode->GetBufferSize(),
        IID_ID3D11ShaderReflection,
        (void**)&pReflectVertexShader
    );

    initializeConstantBuffers((CD3D11Driver *)getVideoDriver(), pReflectVertexShader, EST_VERTEX_SHADER);
    hr = enumInputLayout((CD3D11Driver *)getVideoDriver(), pReflectVertexShader);

    if (FAILED(hr))
    {
        // Release the vertex shader buffer and pixel shader buffer since they are no longer needed.
        SAFE_RELEASE(m_vertexShaderBytecode);
        throw std::runtime_error("invalid shader");
        return false;
    }

    return true;
}

bool irr::video::D3D11HLSLProgram::createGeometryShader(ID3D11Device* device, System::IO::IFileReader* file, const char* entryPoint, const char* shaderModel)
{
    HRESULT hr = S_OK;
    ID3DBlob* errorMessage;
    ID3DBlob* ShaderBuffer = D3D11CompileShader(file, entryPoint, shaderModel, errorMessage);

    // Create the vertex shader from the buffer.
    hr = device->CreateGeometryShader(ShaderBuffer->GetBufferPointer(), ShaderBuffer->GetBufferSize(), NULL, &m_geometryShader);
    if (FAILED(hr))
    {
        throw std::runtime_error("invalid shader");
        return false;
    }

    // get description of precompiled shader
    pReflectGeometryShader = NULL;
    D3DReflect(
        ShaderBuffer->GetBufferPointer(),
        ShaderBuffer->GetBufferSize(),
        IID_ID3D11ShaderReflection,
        (void**)&pReflectGeometryShader
    );

    initializeConstantBuffers((CD3D11Driver *)getVideoDriver(), pReflectGeometryShader, EST_GEOMETRY_SHADER);

    // Release the vertex shader buffer and pixel shader buffer since they are no longer needed.
    SAFE_RELEASE(ShaderBuffer);
    return true;
}

bool irr::video::D3D11HLSLProgram::createPixelShader(ID3D11Device* device, System::IO::IFileReader* file, const char* entryPoint, const char* shaderModel)
{
    HRESULT hr = S_OK;
    ID3DBlob* errorMessage;
    ID3DBlob* ShaderBuffer = D3D11CompileShader(file, entryPoint, shaderModel, errorMessage);

    // Create the vertex shader from the buffer.
    hr = device->CreatePixelShader(ShaderBuffer->GetBufferPointer(), ShaderBuffer->GetBufferSize(), NULL, &m_pixelShader);
    if (FAILED(hr))
    {
        throw std::runtime_error("invalid shader");
        return false;
    }

    // get description of precompiled shader
    pReflectPixelShader = NULL;
    D3DReflect(
        ShaderBuffer->GetBufferPointer(),
        ShaderBuffer->GetBufferSize(),
        IID_ID3D11ShaderReflection,
        (void**)&pReflectPixelShader
    );

    initializeConstantBuffers((CD3D11Driver *)getVideoDriver(), pReflectPixelShader, EST_FRAGMENT_SHADER);

    // Release the vertex shader buffer and pixel shader buffer since they are no longer needed.
    SAFE_RELEASE(ShaderBuffer);
    return true;
}

void irr::video::D3D11HLSLProgram::OutputShaderErrorMessage(ID3D10Blob* errorMessage, HWND hwnd, const char* shaderFilename)
{
    char* compileErrors;
    unsigned long bufferSize, i;
    std::ofstream fout;

    // Get a pointer to the error message text buffer.
    compileErrors = (char*)(errorMessage->GetBufferPointer());

    // Get the length of the message.
    bufferSize = errorMessage->GetBufferSize();

    // Open a file to write the error message to.
    fout.open("shader-error.txt");

    // Write out the error message.
    for (i = 0; i<bufferSize; i++)
    {
        fout << compileErrors[i];
    }

    // Close the file.
    fout.close();

    // Release the error message.
    errorMessage->Release();
    errorMessage = 0;

    // Pop a message up on the screen to notify the user to check the text file for compile errors.
    MessageBox(hwnd, "Error compiling shader.  Check shader-error.txt for message.", shaderFilename, MB_OK);
}

void irr::video::D3D11HLSLProgram::Init()
{
}

void irr::video::D3D11HLSLProgram::ReflParseStruct(SConstantBuffer* buffdesc, irr::video::IShaderVariable* parent, ID3D11ShaderReflectionConstantBuffer * pConstBuffer,
    ID3D11ShaderReflectionType * pParentVariableType, std::vector<irr::video::IShaderVariable*>& Variables, std::string namePrefix, u32 pParentSize)
{
    HRESULT hr = S_OK;

    D3D11_SHADER_BUFFER_DESC pConstBufferDesc;
    D3D11_SHADER_TYPE_DESC pParentTypeDesc;

    pConstBuffer->GetDesc(&pConstBufferDesc);
    if (pParentVariableType)
        pParentVariableType->GetDesc(&pParentTypeDesc);

    Variables.resize(pParentVariableType ? pParentTypeDesc.Members : pConstBufferDesc.Variables);
    for (int m = 0; m < Variables.size(); ++m)
    {
        D3D11_SHADER_VARIABLE_DESC pVariableDesc;
        D3D11_SHADER_TYPE_DESC pTypeDesc;
        ID3D11ShaderReflectionVariable * pVariable = nullptr;

        u32 Offset;
        const char* Name;
        ID3D11ShaderReflectionType * pType;

        if (pParentVariableType)
        {
            Name = pParentVariableType->GetMemberTypeName(m);
            pType = pParentVariableType->GetMemberTypeByIndex(m);
            pType->GetDesc(&pTypeDesc);
            Offset = parent->_getOffset() + pTypeDesc.Offset;
        }
        else
        {
            // description of variable
            ID3D11ShaderReflectionVariable * pVariable = pConstBuffer->GetVariableByIndex(m);
            pVariable->GetDesc(&pVariableDesc);
            Offset = pVariableDesc.StartOffset;
            Name = pVariableDesc.Name;
            pType = pVariable->GetType();
            pType->GetDesc(&pTypeDesc);
        }

        video::E_SHADER_VARIABLE_TYPE eShaderType = getShaderVariableTypeId(pTypeDesc.Type, pTypeDesc.Class);

        SShaderType * _shaderType = CNullShader::GetShaderType(
            eShaderType,
            std::max(1u, pTypeDesc.Elements),
            pTypeDesc.Members,
            pTypeDesc.Rows,
            pTypeDesc.Columns,
            0,
            0,
            0,
            0);

        irr::video::IShaderVariable*& structDecl = Variables[m];

        if (pTypeDesc.Class == D3D_SHADER_VARIABLE_CLASS::D3D_SVC_STRUCT)
        {
            SShaderVariableStruct* structVar = new SShaderVariableStruct();
            structVar->mName = Name; // (namePrefix + Name).c_str();
            structVar->mParent = parent;
            structVar->mBuffer = buffdesc;
            structVar->mOffset = Offset;
            structVar->mLayoutIndex = -1;
            structVar->mType = _shaderType;
            structVar->mIsValid = true;
            structVar->mIsDirty = true;
            structVar->mLoaderData = pType;

            structDecl = structVar;
        }
        else if (pTypeDesc.Class == D3D_SHADER_VARIABLE_CLASS::D3D_SVC_MATRIX_COLUMNS || pTypeDesc.Class == D3D_SHADER_VARIABLE_CLASS::D3D_SVC_MATRIX_ROWS)
        {
            SShaderVariableMatrix* matVar = new SShaderVariableMatrix();
            matVar->mName = Name; //(namePrefix + Name).c_str();
            matVar->mParent = parent;
            matVar->mBuffer = buffdesc;
            matVar->mOffset = Offset;
            matVar->mLayoutIndex = -1;
            matVar->mType = _shaderType;
            matVar->mIsRowMajor = pTypeDesc.Class == D3D_SHADER_VARIABLE_CLASS::D3D_SVC_MATRIX_ROWS;
            matVar->mIsValid = true;
            matVar->mIsDirty = true;

            structDecl = matVar;
        }
        else
        {
            SShaderVariableScalar* var = new SShaderVariableScalar();
            var->mName = Name; //(namePrefix + Name).c_str();
            var->mParent = parent;
            var->mBuffer = buffdesc;
            var->mOffset = Offset;
            var->mLayoutIndex = -1;
            var->mType = _shaderType;
            var->mIsValid = true;
            var->mIsDirty = true;

            structDecl = var;
        }
    }

    std::sort(Variables.begin(), Variables.end(), [](irr::video::IShaderVariable* lhs, irr::video::IShaderVariable* rhs)
    {
        return lhs->_getOffset() < rhs->_getOffset();
    });

    for (int m = 0; m < Variables.size() - 1; ++m)
    {
        auto pVariable = Variables[m];
        auto pVariableNext = Variables[m + 1];

        uint32 stride = pVariableNext->_getOffset() - pVariable->_getOffset();
        uint32 elementSize = stride / pVariable->getType()->Elements;

        pVariable->getType()->ArrayStride = elementSize;
        pVariable->getType()->Stride = stride;
    }

    auto& pVariable = Variables.back();
    uint32 stride = (parent ? (parent->_getOffset() + pParentSize) : pParentSize) - pVariable->_getOffset(); // ToDo
    uint32 elementSize = stride / pVariable->getType()->Elements;

    pVariable->getType()->ArrayStride = elementSize;
    pVariable->getType()->Stride = stride;

    for (int m = 0; m < Variables.size(); ++m)
    {
        auto pVariable = Variables[m];

        if (pVariable->_asStruct() && pVariable->getType()->Members > 0)
        {
            std::string _namePrefix = (namePrefix + pVariable->GetName().c_str()).c_str();
            _namePrefix += '.';

            ReflParseStruct(buffdesc, pVariable, pConstBuffer, static_cast<ID3D11ShaderReflectionType *>(pVariable->_asStruct()->mLoaderData), pVariable->_asStruct()->mVariables, _namePrefix, pVariable->getType()->Stride);
            pVariable->_asStruct()->mLoaderData = nullptr;
        }
    }
}

HRESULT irr::video::D3D11HLSLProgram::initializeConstantBuffers(CD3D11Driver * d3dDevice, ID3D11ShaderReflection* pReflector, E_SHADER_TYPES shaderType)
{
    HRESULT hr = S_OK;

    D3D11_SHADER_DESC ShaderDescriptor;
    pReflector->GetDesc(&ShaderDescriptor);

    // get description of each constant buffer in the shader
    for (u32 i = 0; i < ShaderDescriptor.ConstantBuffers; i++)
    {
        SConstantBuffer* irrCB = new SConstantBuffer(this, shaderType);
        AddConstantBuffer(irrCB);

        // get description of constant buffer
        ID3D11ShaderReflectionConstantBuffer * pConstBuffer = pReflector->GetConstantBufferByIndex(i);
        D3D11_SHADER_BUFFER_DESC pVariable;
        pConstBuffer->GetDesc(&pVariable);

        // get binding description for constant buffer
        D3D11_SHADER_INPUT_BIND_DESC pVariableBind;
        hr = pReflector->GetResourceBindingDescByName(pVariable.Name, &pVariableBind);

        // Create the constant buffer pointer so we can access the vertex shader constant buffer from within this class.
        irrCB->mHwObject = static_cast<CD3D11HardwareBuffer*>(Driver->createHardwareBuffer(E_HARDWARE_BUFFER_TYPE::EHBT_CONSTANTS, E_HARDWARE_BUFFER_ACCESS::EHBA_DYNAMIC, pVariable.Size));
        irrCB->mHostMemory.resize(pVariable.Size);


        irrCB->mOffset = pVariableBind.BindCount;
        irrCB->mBindPoint = pVariableBind.BindPoint;
        irrCB->mName = pVariable.Name;
        irrCB->mBufferType = ESVT_CONSTANT;

        irrCB->mType = CNullShader::GetShaderType(
            E_SHADER_VARIABLE_TYPE::ESVT_BUFFER,
            1,
            pVariable.Variables,
            0,
            1,
            0,
            0,
            0,
            pVariable.Size);

        ReflParseStruct(irrCB, nullptr, pConstBuffer, nullptr, irrCB->mVariables, "", pVariable.Size);
    }

    ///////////////////////////////////////////////////
    // Save description of textures and samplers
    for (u32 i = 0; i != ShaderDescriptor.BoundResources; ++i)
    {
        D3D11_SHADER_INPUT_BIND_DESC inputBindDesc;
        pReflector->GetResourceBindingDesc(i, &inputBindDesc);
        if (inputBindDesc.Type == D3D_SIT_TEXTURE || inputBindDesc.Type == D3D_SIT_SAMPLER)
        {
            SShaderVariableScalar* var = new SShaderVariableScalar();
            var->mName = inputBindDesc.Name;
            var->mParent = nullptr;
            var->mBuffer = nullptr;
            var->mOffset = inputBindDesc.BindPoint;
            var->mLayoutIndex = -1;
            var->mShaderStage = shaderType;
            var->mType = CNullShader::GetShaderType(
                video::E_SHADER_VARIABLE_TYPE::ESVT_SAMPLER,
                1,
                0,
                0,
                1,
                0,
                0,
                0,
                0
            );
            var->mIsValid = true;
            var->mIsDirty = false;
            mTextures.push_back(var);
        }
    }

    return S_OK;
}

HRESULT irr::video::D3D11HLSLProgram::enumInputLayout(CD3D11Driver * d3dDevice, ID3D11ShaderReflection* vertReflect)
{
    inputLayoutArray.clear();
    mVertexInput.clear();

    // http://www.sunandblackcat.com/tipFullView.php?l=eng&topicid=26
    HRESULT hr = S_OK;

    D3D11_SHADER_DESC descVertex;
    vertReflect->GetDesc(&descVertex);

    // save description of input parameters (attributes of vertex shader)
    u32 byteOffset = 0;
    D3D11_SIGNATURE_PARAMETER_DESC input_desc;
    for (u32 i = 0; i < descVertex.InputParameters; ++i)
    {
        // get description of input parameter
        vertReflect->GetInputParameterDesc(i, &input_desc);
        
        // fill element description to create input layout later
        D3D11_INPUT_ELEMENT_DESC ie;
        ie.SemanticName = input_desc.SemanticName;
        ie.SemanticIndex = input_desc.SemanticIndex;
        ie.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
        ie.InputSlot = 0; // can't reflect
        ie.InputSlotClass = D3D11_INPUT_PER_VERTEX_DATA;  // can't reflect
        ie.InstanceDataStepRate = 0;  // can't reflect
        ie.AlignedByteOffset = byteOffset;

        u32 componentCount = core::math::NumberOfSetBits(input_desc.Mask);
        byteOffset += (4 * componentCount);

        video::E_SHADER_VARIABLE_TYPE eShaderType;
        if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_UINT32)
            eShaderType = video::E_SHADER_VARIABLE_TYPE::ESVT_UINT;
        else if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_SINT32)
            eShaderType = video::E_SHADER_VARIABLE_TYPE::ESVT_INT;
        else
            eShaderType = video::E_SHADER_VARIABLE_TYPE::ESVT_FLOAT;

        // determine correct format of input parameter and offset
        if (componentCount == 1)
        {
            if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_UINT32)
            {
                ie.Format = DXGI_FORMAT_R32_UINT;
            }
            else if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_SINT32)
            {
                ie.Format = DXGI_FORMAT_R32_SINT;
            }
            else if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_FLOAT32)
            {
                ie.Format = DXGI_FORMAT_R32_FLOAT;
            }
        }
        else if (componentCount == 2)
        {
            if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_UINT32)
            {
                ie.Format = DXGI_FORMAT_R32G32_UINT;
            }
            else if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_SINT32)
            {
                ie.Format = DXGI_FORMAT_R32G32_SINT;
            }
            else if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_FLOAT32)
            {
                ie.Format = DXGI_FORMAT_R32G32_FLOAT;
            }
        }
        else if (componentCount == 3)
        {
            if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_UINT32)
            {
                ie.Format = DXGI_FORMAT_R32G32B32_UINT;
            }
            else if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_SINT32)
            {
                ie.Format = DXGI_FORMAT_R32G32B32_SINT;
            }
            else if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_FLOAT32)
            {
                ie.Format = DXGI_FORMAT_R32G32B32_FLOAT;
            }
        }
        else if (componentCount == 4)
        {
            if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_UINT32)
            {
                ie.Format = DXGI_FORMAT_R32G32B32A32_UINT;
            }
            else if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_SINT32)
            {
                ie.Format = DXGI_FORMAT_R32G32B32A32_SINT;
            }
            else if (input_desc.ComponentType == D3D_REGISTER_COMPONENT_FLOAT32)
            {
                ie.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
            }
        }

        SShaderVariableScalar* var = new SShaderVariableScalar();
        var->mName = "";
        var->mParent = nullptr;
        var->mBuffer = nullptr;
        var->mOffset = byteOffset;
        var->mLayoutIndex = i;
        var->mSemantic = input_desc.SemanticName;
        var->mSemanticIndex = input_desc.SemanticIndex;

        var->mType = CNullShader::GetShaderType(
            eShaderType,
            1,
            0,
            0,
            componentCount,
            0,
            0,
            0,
            (4 * componentCount)
        );

        var->mIsValid = true;
        var->mIsDirty = false;
        mVertexInput.push_back(var);

        inputLayoutArray.push_back(ie);
    }

    return hr;
}
