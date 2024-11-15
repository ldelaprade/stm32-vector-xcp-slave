
# Include post actions

if  grep 'postCubeMXAction.cmake' CMakeLists.txt ;then
   echo "# Commands for bin generation already included"
else
    echo "# ----------------------------" >> CMakeLists.txt
    echo "# Commands for bin generation " >> CMakeLists.txt
    echo "# ----------------------------" >> CMakeLists.txt
    echo "include(\"postCubeMXAction.cmake\")"  >> CMakeLists.txt
    echo "# ----------------------------" >> CMakeLists.txt
fi


if [ -f "STM32F767ZITx_FLASH.ld" ]; then
    if [ -f "STM32F767ZITx_FLASH.ld.ref" ]; then
        mv STM32F767ZITx_FLASH.ld  STM32F767ZITx_FLASH.ld.buggy
        cp STM32F767ZITx_FLASH.ld.ref STM32F767ZITx_FLASH.ld
        echo "Using STM32F767ZITx_FLASH.ld.ref."
    fi
fi

