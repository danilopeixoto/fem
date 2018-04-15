!include "MUI2.nsh"

!define OUTPUT_DIRECTORY "build"
!define DISTRIBUTION_DIRECTORY "..\build\release"

!define MAYA_VERSION "2018"
!define MAYA_MODULE_FILE "$COMMONFILES64\Autodesk Shared\Modules\Maya\${MAYA_VERSION}\${PRODUCT_BASENAME}.mod"

!define PRODUCT_NAME "FEM"
!define UNINSTALL_NAME "Uninstall"

!define PRODUCT_BASENAME "fem"
!define UNINSTALL_BASENAME "uninstall"

!define PRODUCT_DESCRIPTION "FEM"
!define PRODUCT_VERSION "1.0.0"
!define PRODUCT_AUTHOR "Danilo Ferreira"
!define PRODUCT_COPYRIGHT "Copyright (c) 2018, Danilo Ferreira. All rights reserved."

!define PRODUCT_LANGUAGE "English"
!define PRODUCT_LICENSE_FILE "docs\license.rtf"

!define PRODUCT_INSTALLER_NAME "${PRODUCT_BASENAME}-${PRODUCT_VERSION}-maya-${MAYA_VERSION}-setup.exe"
!define PRODUCT_UNINSTALL_NAME "${UNINSTALL_BASENAME}.exe"

!define PRODUCT_INSTALLER_ICON "icons\setup.ico"
!define PRODUCT_INSTALLER_BANNER "images\banner.bmp"

!define UNINSTALL_SHORTCUT_NAME "${UNINSTALL_NAME}.lnk"

!define PRODUCT_INSTALL_DIRECTORY "$PROGRAMFILES64\${PRODUCT_NAME}"
!define PRODUCT_START_MENU_DIRECTORY "$SMPROGRAMS\${PRODUCT_NAME}"

!define VERSION_START_MENU_DIRECTORY "${PRODUCT_START_MENU_DIRECTORY}\${MAYA_VERSION}"

!define REGISTER_ROOT_KEY "HKLM"
!define REGISTER_UNINSTALL_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${PRODUCT_NAME}"

!system 'md "${OUTPUT_DIRECTORY}"'

Name "${PRODUCT_NAME}"
BrandingText " "
OutFile "${OUTPUT_DIRECTORY}\${PRODUCT_INSTALLER_NAME}"
InstallDir "${PRODUCT_INSTALL_DIRECTORY}\${MAYA_VERSION}"
SetCompressor zlib

VIProductVersion "${PRODUCT_VERSION}.0"
VIAddVersionKey "ProductName" "${PRODUCT_NAME}"
VIAddVersionKey "ProductVersion" "${PRODUCT_VERSION}"
VIAddVersionKey "CompanyName" "${PRODUCT_AUTHOR}"
VIAddVersionKey "LegalCopyright" "${PRODUCT_COPYRIGHT}"
VIAddVersionKey "FileDescription" "${PRODUCT_DESCRIPTION}"
VIAddVersionKey "FileVersion" "${PRODUCT_VERSION}"

!define MUI_ABORTWARNING
!define MUI_ICON "${PRODUCT_INSTALLER_ICON}"
!define MUI_UNICON "${PRODUCT_INSTALLER_ICON}"
!define MUI_WELCOMEFINISHPAGE_BITMAP "${PRODUCT_INSTALLER_BANNER}"
!define MUI_UNWELCOMEFINISHPAGE_BITMAP "${PRODUCT_INSTALLER_BANNER}"

!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_LICENSE "${PRODUCT_LICENSE_FILE}"
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_WELCOME
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_UNPAGE_FINISH

!insertmacro MUI_LANGUAGE "${PRODUCT_LANGUAGE}"

Section Installer
    SetOutPath "$INSTDIR"
    
    File /r "${DISTRIBUTION_DIRECTORY}\*"
    File "${PRODUCT_LICENSE_FILE}"
SectionEnd

Section -AdditionalIcons
    CreateDirectory "${VERSION_START_MENU_DIRECTORY}"
    CreateShortCut "${VERSION_START_MENU_DIRECTORY}\${UNINSTALL_SHORTCUT_NAME}" "$INSTDIR\${PRODUCT_UNINSTALL_NAME}"
SectionEnd

Section -Post
    WriteUninstaller "$INSTDIR\${PRODUCT_UNINSTALL_NAME}"
    
    WriteRegStr "${REGISTER_ROOT_KEY}" "${REGISTER_UNINSTALL_KEY}" "DisplayName" "${PRODUCT_NAME}"
    WriteRegStr "${REGISTER_ROOT_KEY}" "${REGISTER_UNINSTALL_KEY}" "DisplayVersion" "${PRODUCT_VERSION}"
    WriteRegStr "${REGISTER_ROOT_KEY}" "${REGISTER_UNINSTALL_KEY}" "DisplayIcon" "$INSTDIR\${PRODUCT_UNINSTALL_NAME}"
    WriteRegStr "${REGISTER_ROOT_KEY}" "${REGISTER_UNINSTALL_KEY}" "Publisher" "${PRODUCT_AUTHOR}"
    WriteRegStr "${REGISTER_ROOT_KEY}" "${REGISTER_UNINSTALL_KEY}" "UninstallString" "$INSTDIR\${PRODUCT_UNINSTALL_NAME}"
    
    Call createModuleFile
SectionEnd

Function createModuleFile
FileOpen $0 "${MAYA_MODULE_FILE}" w
FileWrite $0 "+ ${PRODUCT_BASENAME} ${PRODUCT_VERSION} $INSTDIR$\n"
FileWrite $0 "PATH +:= bin"
FileClose $0
FunctionEnd

Section Uninstall
    RMDir /r /REBOOTOK "$INSTDIR"
    RMDir /r /REBOOTOK "${VERSION_START_MENU_DIRECTORY}"
    
    RMDir /REBOOTOK "${PRODUCT_INSTALL_DIRECTORY}"
    RMDir /REBOOTOK "${PRODUCT_START_MENU_DIRECTORY}"

    Delete /REBOOTOK "${MAYA_MODULE_FILE}"

    DeleteRegKey "${REGISTER_ROOT_KEY}" "${REGISTER_UNINSTALL_KEY}"
SectionEnd