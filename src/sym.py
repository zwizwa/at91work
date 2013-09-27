# LICENSE: GPL2
# (c) 2013 Tom Schouten <tom@getbeep.com>

# Central place to store all symbolic<->numeric tag conversions.

# Build bi-directional lookup with some automatic conversion

# Sentinel
class empty_table:
    def __getattr__(self, key):
        raise Exception("symbolic key not found: %s" % key)
    def __getitem__(self, key):
        raise Exception("numeric key not found: %s" % key)

class sym_table:
    def __init__(self, num_to_sym, parent = empty_table()):
        self.parent = parent
        self.n2s = num_to_sym
        self.s2n = dict([(s,n) for n,s in num_to_sym.items()])
    # table.key
    def __getattr__(self, key):
        try:
            return self.s2n[key]
        except:
            return self.parent.__getattr__(key)
    # table[key]
    def __getitem__(self, key):
        try:
            return self.n2s[key]
        except:
            return self.parent.__getitem__(key)


# ETSI TS 102 221 - 10.2.1 Status conditions returned by the UICC
SW1 = sym_table({
    0x91 : 'OK_INFO_PROACTIVE_COMMAND',
    0x92 : 'OK_INFO_DATA_TRANSFER',
})
SW = sym_table({
    0x9000 : 'OK',
    0x9300 : 'TOOLKIT_BUSY',
    0x6282 : 'FILE_INVALID',
    0x6283 : 'FILE_INVALID',
    0x6700 : 'WRONG_LENGTH',
    0x6981 : 'INCOMPATIBLE_FILE_STRUCTURE',
    0x6982 : 'SECURITY_STATUS',
    0x6985 : 'CONDITIONS_OF_USE_NOT_SATISFIED',
    0x6A82 : 'FILE_NOT_FOUND',
})
            

iso7816 = sym_table({
    0x04 : 'DEACTIVATE_FILE',
    0x0E : 'ERASE_BINARY',
    0x10 : 'TERMINAL_PROFILE',
    0x12 : 'FETCH',
    0x14 : 'TERMINAL_RESPONSE',
    0x20 : 'VERIFY',
    0x24 : 'CHANGE_PIN',
    0x26 : 'DISABLE_PIN',
    0x28 : 'ENABLE_PIN',
    0x2C : 'UNBLOCK_PIN',
    0x32 : 'INCREASE',
    0x44 : 'ACTIVATE_FILE',
    0x70 : 'MANAGE_CHANNEL',
    0x73 : 'MANAGE_SECURE_CHANNEL',
    0x75 : 'TRANSACT_DATA',
    0x82 : 'EXTERNAL_AUTHENTICATE',
    0x84 : 'GET_CHALLENGE',
    0x88 : 'INTERNAL_AUTHENTICATE',
    0xA2 : 'SEARCH_RECORD',
    0xA4 : 'SELECT_FILE',
    0xAA : 'TERMINAL_CAPABILITY',
    0xB0 : 'READ_BINARY',
    0xB2 : 'READ_RECORD',
    0xC0 : 'GET_RESPONSE',
    0xC2 : 'ENVELOPE',
    0xCB : 'RETRIEVE_DATA',
    0xCA : 'GET_DATA',
    0xD0 : 'WRITE_BINARY',
    0xD2 : 'WRITE_RECORD',
    0xD6 : 'UPDATE_BINARY',
    0xDA : 'PUT_DATA',
    0xDB : 'SET_DATA',
    0xDC : 'UPDATE_DATA',
    0xE2 : 'APPEND_RECORD',
    0xF2 : 'STATUS',
})

SIM_FID = sym_table({
    0x3F00 : 'MF',
    0x7F10 : 'DF_TELECOM',
    0x7F20 : 'DF_GSM',
    0x7F21 : 'DF_DCS_1800',
    0x7F22 : 'DF_IS_41',
    0x7F23 : 'DF_FP_CTS',
    0x7F80 : 'DF_PDC',
    0x7F90 : 'DF_TETRA',
    0x7F24 : 'DF_TIA_EIA_136',
    0x7F25 : 'DF_TIA_EIA_95',
    0x5F50 : 'DF_GRAPHICS',
    0x5F30 : 'DF_IRIDIUM',
    0x5F31 : 'DF_GLOBALSTAR',
    0x5F32 : 'DF_ICO',
    0x5F33 : 'DF_ACES',
    0x5F40 : 'DF_PCS_1900',
    0x5F60 : 'DF_CTS',
    0x5F70 : 'DF_SOLSA',
    0x5F40 : 'DF_TIA_EIA_553',
    0x5F3C : 'DF_MEXE',
    0x2FE2 : 'EF_ICCID',
    0x2F05 : 'EF_ELP',
    0x6F3A : 'EF_ADN',
    0x6F3B : 'EF_FDN',
    0x6F3C : 'EF_SMS',
    0x6F3D : 'EF_CCP',
    0x6F40 : 'EF_MSISDN',
    0x6F42 : 'EF_SMSP',
    0x6F43 : 'EF_SMSS',
    0x6F44 : 'EF_LND',
    0x6F49 : 'EF_SDN',
    0x6F4A : 'EF_EXT1',
    0x6F4B : 'EF_EXT2',
    0x6F4C : 'EF_EXT3',
    0x6F4D : 'EF_BDN',
    0x6F4E : 'EF_EXT4',
    0x6F47 : 'EF_SMSR',
    0x6F4F : 'EF_ECCP',
    0x6F58 : 'EF_CMI',
    0x4F20 : 'EF_IMG',
    0x6F05 : 'EF_LP',
    0x6F07 : 'EF_IMSI',
    0x6F16 : 'EF_CPHS', # nonstandard / Common PCN Handset Specification?
    0x6F20 : 'EF_KC',
    0x6F30 : 'EF_PLMNSEL',
    0x6F31 : 'EF_HPLMN',
    0x6F37 : 'EF_ACMMAX',
    0x6F38 : 'EF_SST',
    0x6F39 : 'EF_ACM',
    0x6F3E : 'EF_GID1',
    0x6F3F : 'EF_GID2',
    0x6F46 : 'EF_SPN',
    0x6F41 : 'EF_PUCT',
    0x6F45 : 'EF_CBMI',
    0x6F74 : 'EF_BCCH',
    0x6F78 : 'EF_ACC',
    0x6F7B : 'EF_FPLMN',
    0x6F7E : 'EF_LOCI',
    0x6FAD : 'EF_AD',
    0x6FAE : 'EF_PHASE',
    0x6FB1 : 'EF_VGCS',
    0x6FB2 : 'EF_VGCSS',
    0x6FB3 : 'EF_VBS',
    0x6FB4 : 'EF_VBSS',
    0x6FB5 : 'EF_EMLPP',
    0x6FB6 : 'EF_AAEM',
    0x6F48 : 'EF_CBMID',
    0x6FB7 : 'EF_ECC',
    0x6F50 : 'EF_CBMIR',
    0x6F2C : 'EF_DCK',
    0x6F32 : 'EF_CNL',
    0x6F51 : 'EF_NIA',
    0x6F52 : 'EF_KCGPRS',
    0x6F53 : 'EF_LOCIGPRS',
    0x6F54 : 'EF_SUME',
    0x6F60 : 'EF_PLMNWACT',
    0x6F61 : 'EF_OPLMNWACT',
    0x6F62 : 'EF_HPLMNWACT',
    0x6F63 : 'EF_CPBCCH',
    0x6F64 : 'EF_INVSCAN',
    0x4F30 : 'EF_SAI',
    0x4F31 : 'EF_SLL',
    0x4F80 : 'EF_SID',
    0x4F81 : 'EF_GPI',
    0x4F82 : 'EF_IPC',
    0x4F83 : 'EF_COUNT',
    0x4F84 : 'EF_NSID',
    0x4F85 : 'EF_PSID',
    0x4F86 : 'EF_NETSEL',
    0x4F87 : 'EF_SPL',
    0x4F88 : 'EF_MIN',
    0x4F89 : 'EF_ACCOLC',
    0x4F8A : 'EF_FC1',
    0x4F8B : 'EF_S_ESN',
    0x4F8C : 'EF_CSID',
    0x4F8D : 'EF_REG_THRESH',
    0x4F8E : 'EF_CCCH',
    0x4F8F : 'EF_LDCC',
    0x4F90 : 'EF_GSM_RECON',
    0x4F91 : 'EF_AMPS_2_GSM',
    0x4F93 : 'EF_AMPS_UI',
    0x4F40 : 'EF_MEXE_ST',
    0x4F41 : 'EF_ORPK',
    0x4F42 : 'EF_ARPK',
    0x4F43 : 'EF_TPRPK',
})

USIM_FID = sym_table({
    0x2F00 : 'EF_DIR',
    0x2F06 : 'EF_ARR',
    0x3F00 : 'MF',
    0x7FFF : 'ADF',
    0x6F05 : 'EF_LI',
    0x6F07 : 'EF_IMSI',
    0x6F08 : 'EF_KEYS',
    0x6F09 : 'EF_KEYSPS',
    0x6F60 : 'EF_PLMNWACT',
    0x6F31 : 'EF_HPPLMN',
    0x6F37 : 'EF_ACMMAX',
    0x6F38 : 'EF_UST',
    0x6F39 : 'EF_ACM',
    0x6F3E : 'EF_GID1',
    0x6F3F : 'EF_GID2',
    0x6F46 : 'EF_SPN',
    0x6F41 : 'EF_PUCT',
    0x6F45 : 'EF_CBMI',
    0x6F78 : 'EF_ACC',
    0x6F7B : 'EF_FPLMN',
    0x6F7E : 'EF_LOCI',
    0x6FAD : 'EF_AD',
    0x6F48 : 'EF_CBMID',
    0x6FB7 : 'EF_ECC',
    0x6F50 : 'EF_CBMIR',
    0x6F73 : 'EF_PSLOCI',
    0x6F3B : 'EF_FDN',
    0x6F3C : 'EF_SMS',
    0x6F40 : 'EF_MSISDN',
    0x6F42 : 'EF_SMSP',
    0x6F43 : 'EF_SMSS',
    0x6F49 : 'EF_SDN',
    0x6F4B : 'EF_EXT2',
    0x6F4C : 'EF_EXT3',
    0x6F47 : 'EF_SMSR',
    0x6F80 : 'EF_ICI',
    0x6F81 : 'EF_OCI',
    0x6F82 : 'EF_ICT',
    0x6F83 : 'EF_OCT',
    0x6F4E : 'EF_EXT5',
    0x6F4F : 'EF_CCP2',
    0x6FB5 : 'EF_EMLPP',
    0x6FB6 : 'EF_AAEM',
    0x6FC3 : 'EF_HIDDENKEY',
    0x6F4D : 'EF_BDN',
    0x6F55 : 'EF_EXT4',
    0x6F58 : 'EF_CMI',
    0x6F56 : 'EF_EST',
    0x6F57 : 'EF_ACL',
    0x6F2C : 'EF_DCK',
    0x6F32 : 'EF_CNL',
    0x6F5B : 'EF_START_HFN',
    0x6F5C : 'EF_THRESHOLD',
    0x6F61 : 'EF_OPLMNWACT',
    0x6F62 : 'EF_HPLMNWACT',
    0x6F06 : 'EF_ARR',
    0x6FC4 : 'EF_NETPAR',
    0x6FC5 : 'EF_PNN',
    0x6FC6 : 'EF_OPL',
    0x6FC7 : 'EF_MBDN',
    0x6FC8 : 'EF_EXT6',
    0x6FC9 : 'EF_MBI',
    0x6FCA : 'EF_MWIS',
    0x6FCB : 'EF_CFIS',
    0x6FCC : 'EF_EXT7',
    0x6FCD : 'EF_SPDI',
    0x6FCE : 'EF_MMSN',
    0x6FCF : 'EF_EXT8',
    0x6FD0 : 'EF_MMSICP',
    0x6FD1 : 'EF_MMSUP',
    0x6FD2 : 'EF_MMSUCP',
    0x6FD3 : 'EF_NIA',
    0x6FB1 : 'EF_VGCS',
    0x6FB2 : 'EF_VGCSS',
    0x6FB3 : 'EF_VBS',
    0x6FB4 : 'EF_VBSS',
    0x6FD4 : 'EF_VGCSCA',
    0x6FD5 : 'EF_VBSCA',
    0x6FD6 : 'EF_GBABP',
    0x6FD7 : 'EF_MSK',
    0x6FD8 : 'EF_MUK',
    0x5F3A : 'DF_PHONEBOOK',
    0x5F3B : 'DF_GSM_ACCESS',
    0x5F3C : 'DF_MEXE',
    0x5F40 : 'DF_WLAN',
    0x5F70 : 'DF_SOLSA',
    0x4F30 : 'EF_SAI',
    0x4F31 : 'EF_SLL',
    0x4F30 : 'EF_PBR',
    0x4F22 : 'EF_PSC',
    0x4F23 : 'EF_CC',
    0x4F24 : 'EF_PUID',
    0x4F20 : 'EF_KC',
    0x4F52 : 'EF_KCGPRS',
    0x4F63 : 'EF_CPBCCH',
    0x4F64 : 'EF_INVSCAN',
    0x4F40 : 'EF_MEXEST',
    0x4F41 : 'EF_ORPK',
    0x4F42 : 'EF_ARPK',
    0x4F43 : 'EF_TRPRK',
    0x4F41 : 'EF_PSEUDO',
    0x4F42 : 'EF_UPLMNWLAN',
    0x4F43 : 'EF_OPLMNWLAN',
    0x4F44 : 'EF_UWSIDL',
    0x4F45 : 'EF_OWSIDL',
    0x4F46 : 'EF_WRI',
    0x6FE0 : 'EF_ICE_DN',
    0x6FE1 : 'EF_ICE_FF',
    0x6F3A : 'EF_ADN',
    0x6F4A : 'EF_EXT1',
    0x6F4F : 'EF_ECCP',
    0x6F54 : 'EF_SUME',
    0x5F50 : 'DF_GRAPHICS',
    0x5F3B : 'DF_MULTIMEDIA',
    0x4F20 : 'EF_IMG',
    0x4F47 : 'EF_MML',
    0x4F48 : 'EF_MMDF',
    0x4F81 : 'EF_ACSGL',
    0x4F82 : 'EF_CSGI',
    0x4F82 : 'EF_CSGT',
    0x4F83 : 'EF_HNBN',
    0x4F84 : 'EF_OCSGL',
    0x4F85 : 'EF_OCSGT',
    0x4F86 : 'EF_OHNBN',
},
parent = SIM_FID
)

# ETSI TS 102 223 - 9.4
proactive_command = sym_table({
    0x03 : 'POLL_INTERVAL',
    0x13 : 'SEND_SHORT_MESSAGE',
    0x16 : 'GEOGRAPHICAL_LOCATION_REQUEST',
})
    
# ETSI TS 101 220 - 7.2 Assigned TLV tag values
# Card application toolkit templates
cat = sym_table({
    0xD0 : 'PROACTIVE_COMMAND',
    0xD1 : 'SMS_PP_DOWNLOAD',
})

# ETSI TS 101 220 - 7.2 Assigned TLV tag values
# Card application toolkit data objects
cat_data = sym_table({
    0x81 : 'COMMAND_DETAILS',
    0x82 : 'DEVICE_IDENTITY',
    0x0B : 'SMS_PDU',
})

def test():
    assert iso7816[0xF2]  == 'STATUS'
    assert iso7816.STATUS == 0xF2
    assert SIM_FID[0x7F10] == 'DF_TELECOM'
    assert SIM_FID.DF_TELECOM == 0x7F10
    assert USIM_FID[0x7F10] == 'DF_TELECOM'  # delegation
    assert USIM_FID.DF_TELECOM == 0x7F10     # delegation
    try:
        USIM_FID[-1]
    except Exception as e:
        pass
    try:
        USIM_FID.bad_key
    except Exception as e:
        pass
    print "test OK"

if __name__ == '__main__':
    test()

