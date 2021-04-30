/***********************************************
 * 向485传感器发送查询命令
 * -------------------------------------------------------
 * | 01 | 03  |  00  |  00  |  00  |  01  |  00  |  00   |
 * | id | cmd | addh | addl | lenh | lenl | crcl | crch  | 
 * -------------------------------------------------------
 */
void Om_Func_Handle_Sensor_Data(void)
{
    
    u16 u16Index;
    u8  pu8Buffer[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00 };
    u8  pu8BufferStr[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    u32 u32Length;

    if((g_struOmSysConfig.u16DeviceType == OM_DEVICE_TYPE_FENGSU485)    
        ||(g_struOmSysConfig.u16DeviceType == OM_DEVICE_TYPE_FENGXIANG485)){
        pu8Buffer[1] = 0x03;
        pu8Buffer[5] = 0x01;
    }else if((g_struOmSysConfig.u16DeviceType == OM_DEVICE_TYPE_ZIWAIXIAN485) ){
        pu8Buffer[1] = 0x03;
        pu8Buffer[3] = 0x12;
        pu8Buffer[5] = 0x01;
    }


    for (u16Index = 0; u16Index < g_struOmSysConfig.u16Sensor485Num; )
    {
        /*循环遍历传感器数据*/
        if(g_usart3Lock==1){

            break;
        }else  
        {
            u16Index++;

        }
        pu8Buffer[0] = u16Index;
        ModBusCRC16(pu8Buffer,6);
        Dd_Usart3_Send(pu8Buffer,8);
        hex2str(pu8BufferStr,pu8Buffer,8);
        SSP_SYSLOG(SYSLOG_TRACE,"send 485cmd %s\r\n", pu8BufferStr);
        Ssp_Delay(10);
    }

}