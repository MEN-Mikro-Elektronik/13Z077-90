<?xml version='1.0' encoding='ISO-8859-1' standalone='yes'?>
<tagfile>
  <compound kind="page">
    <filename>index</filename>
    <title>Linux native driver for Ethernet IP cores</title>
    <name>index</name>
  </compound>
  <compound kind="file">
    <name>men_16z077_doc.c</name>
    <path>/opt/menlinux/DRIVERS/ETH_16Z077/DOC/</path>
    <filename>men__16z077__doc_8c</filename>
  </compound>
  <compound kind="file">
    <name>men_16z077_eth.c</name>
    <path>/opt/menlinux/DRIVERS/ETH_16Z077/DRIVER/</path>
    <filename>men__16z077__eth_8c</filename>
    <class kind="struct">z77_i2c_msg_st</class>
    <class kind="struct">z77_private</class>
    <member kind="define">
      <type>#define</type>
      <name>I2C_M_TEN</name>
      <anchor>a27</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>I2C_M_WR</name>
      <anchor>a28</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>I2C_M_RD</name>
      <anchor>a29</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>I2C_M_NOSTART</name>
      <anchor>a30</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_open</name>
      <anchor>a55</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_send_packet</name>
      <anchor>a56</anchor>
      <arglist>(struct sk_buff *skb, struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>irqreturn_t</type>
      <name>z77_irq</name>
      <anchor>a57</anchor>
      <arglist>(int irq, void *dev_id)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_close</name>
      <anchor>a58</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>net_device_stats *</type>
      <name>z77_get_stats</name>
      <anchor>a59</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_tx_timeout</name>
      <anchor>a60</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_rx_err</name>
      <anchor>a61</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_tx_err</name>
      <anchor>a62</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_poll</name>
      <anchor>a63</anchor>
      <arglist>(struct napi_struct *napi, int budget)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_phy_reset</name>
      <anchor>a65</anchor>
      <arglist>(struct net_device *dev, u8 phyAddr)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_phy_identify</name>
      <anchor>a66</anchor>
      <arglist>(struct net_device *dev, u8 phyAddr)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_phy_init</name>
      <anchor>a67</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_init_phymode</name>
      <anchor>a68</anchor>
      <arglist>(struct net_device *dev, u8 phyAddr)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_pass_packet</name>
      <anchor>a69</anchor>
      <arglist>(struct net_device *dev, unsigned int idx)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_reset</name>
      <anchor>a72</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_ioctl</name>
      <anchor>a73</anchor>
      <arglist>(struct net_device *dev, struct ifreq *ifr, int cmd)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_store_mac</name>
      <anchor>a85</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_set_rx_mode</name>
      <anchor>a86</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_bd_setup</name>
      <anchor>a105</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_do_autonegotiation</name>
      <anchor>a106</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>cleanup_card</name>
      <anchor>a107</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_reset_task</name>
      <anchor>a108</anchor>
      <arglist>(struct work_struct *work)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>u32</type>
      <name>z77_get_oldest_frame</name>
      <anchor>a109</anchor>
      <arglist>(u32 rx0, u32 rx1, u32 *nrframes)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_i2c_delay</name>
      <anchor>a110</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_i2c_freeBus</name>
      <anchor>a111</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_i2c_start</name>
      <anchor>a112</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_i2c_stop</name>
      <anchor>a113</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_i2c_checkAckn</name>
      <anchor>a114</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_i2c_setAckn</name>
      <anchor>a115</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_i2c_notAckn</name>
      <anchor>a116</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_i2c_sendByte</name>
      <anchor>a117</anchor>
      <arglist>(struct net_device *dev, u8 val)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_i2c_readByte</name>
      <anchor>a118</anchor>
      <arglist>(struct net_device *dev, u8 *valP)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>z77_i2c_read_msg</name>
      <anchor>a119</anchor>
      <arglist>(struct net_device *dev, struct z77_i2c_msg_st *msg, int stop)</arglist>
    </member>
    <member kind="function">
      <type>u32</type>
      <name>z77_i2c_write_msg</name>
      <anchor>a120</anchor>
      <arglist>(struct net_device *dev, const struct z77_i2c_msg_st *msg, int stop)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>ssize_t</type>
      <name>z77_eeprod_mac_show</name>
      <anchor>a123</anchor>
      <arglist>(struct device *dev, struct device_attribute *attr, char *buf)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>ssize_t</type>
      <name>z77_eeprod_mac_store</name>
      <anchor>a124</anchor>
      <arglist>(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>z77_get_mac_from_board_id</name>
      <anchor>a126</anchor>
      <arglist>(u8 *mac)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>chipset_init</name>
      <anchor>a127</anchor>
      <arglist>(struct net_device *dev, u32 first_init)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void</type>
      <name>z77_tx</name>
      <anchor>a128</anchor>
      <arglist>(struct net_device *dev)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>men_16z077_probe</name>
      <anchor>a129</anchor>
      <arglist>(CHAMELEON_UNIT_T *chu)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int</type>
      <name>men_16z077_remove</name>
      <anchor>a130</anchor>
      <arglist>(CHAMELEON_UNIT_T *chu)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>void __exit</type>
      <name>men_16z077_cleanup</name>
      <anchor>a131</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>int __init</type>
      <name>men_16z077_init</name>
      <anchor>a132</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>ethtool_ops</type>
      <name>z77_ethtool_ops</name>
      <anchor>a47</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>phy_device_tbl</name>
    <filename>structphy__device__tbl.html</filename>
    <member kind="variable">
      <type>unsigned int</type>
      <name>ident</name>
      <anchor>o0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const char *</type>
      <name>name</name>
      <anchor>o1</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>z077_bd</name>
    <filename>structz077__bd.html</filename>
    <member kind="variable">
      <type>unsigned short</type>
      <name>BdLen</name>
      <anchor>o0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>unsigned short</type>
      <name>BdStat</name>
      <anchor>o1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>void *</type>
      <name>BdAddr</name>
      <anchor>o2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>dma_addr_t</type>
      <name>hdlDma</name>
      <anchor>o3</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>z077_reg_info</name>
    <filename>structz077__reg__info.html</filename>
    <member kind="variable">
      <type>const char *</type>
      <name>name</name>
      <anchor>o0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>unsigned int</type>
      <name>addr</name>
      <anchor>o1</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>z77_private</name>
    <filename>structz77__private.html</filename>
    <member kind="variable">
      <type>long</type>
      <name>open_time</name>
      <anchor>o0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>long</type>
      <name>flags</name>
      <anchor>o1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>instance</name>
      <anchor>o2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>phymode</name>
      <anchor>o3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>instCount</name>
      <anchor>o4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>nCurrTbd</name>
      <anchor>o5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>txIrq</name>
      <anchor>o6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>modCode</name>
      <anchor>o7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>unsigned long</type>
      <name>bdBase</name>
      <anchor>o8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>pollInProgress</name>
      <anchor>o10</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>bdOff</name>
      <anchor>o11</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>tbdOff</name>
      <anchor>o12</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>rbdOff</name>
      <anchor>o13</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>serialnr</name>
      <anchor>o14</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>board</name>
      <anchor>o15</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>work_struct</type>
      <name>reset_task</name>
      <anchor>o16</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>timer_list</type>
      <name>timer</name>
      <anchor>o17</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>prev_linkstate</name>
      <anchor>o18</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u8</type>
      <name>mcast_lst</name>
      <anchor>o19</anchor>
      <arglist>[MAX_MCAST_LST][MAC_ADDR_LEN]</arglist>
    </member>
    <member kind="variable">
      <type>Z077_BD</type>
      <name>txBd</name>
      <anchor>o20</anchor>
      <arglist>[Z077_TBD_NUM]</arglist>
    </member>
    <member kind="variable">
      <type>Z077_BD</type>
      <name>rxBd</name>
      <anchor>o21</anchor>
      <arglist>[Z077_RBD_NUM]</arglist>
    </member>
    <member kind="variable">
      <type>net_device_stats</type>
      <name>stats</name>
      <anchor>o22</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>spinlock_t</type>
      <name>lock</name>
      <anchor>o23</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>pci_dev *</type>
      <name>pdev</name>
      <anchor>o24</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>mii_if_info</type>
      <name>mii_if</name>
      <anchor>o25</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>spinlock_t</type>
      <name>mii_lock</name>
      <anchor>o26</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>u32</type>
      <name>msg_enable</name>
      <anchor>o27</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>napi_struct</type>
      <name>napi</name>
      <anchor>o28</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="page">
    <name>16z077dummy</name>
    <title>MEN logo</title>
    <filename>16z077dummy</filename>
  </compound>
  <compound kind="group">
    <name>_NETOPS_FUNC</name>
    <title>ethtool support functions</title>
    <filename>group____NETOPS__FUNC.html</filename>
  </compound>
  <compound kind="group">
    <name>_Z077MACS</name>
    <title>board specific MAC offsets</title>
    <filename>group____Z077MACS.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z77_MAC_OFFS_F11S</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z77_MAC_OFFS_F218</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z77_MAC_OFFS_P511</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z77_MAC_P511_IF2OFF</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z77_MAC_OFFS_P513</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z77_MAC_P513_IF2OFF</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_Z077BASES</name>
    <title>Important basic defines for following Macros</title>
    <filename>group____Z077BASES.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z077_BASE</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_BDBASE</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_BD_OFFS</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TBD_OFF</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>RBD_OFF</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z77_ETHBUF_SIZE</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z77_CFGREG_SIZE</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>MY_TX_TIMEOUT</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_BASE</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z77_PACKLEN_DEFAULT</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_Z077_REGISTERS</name>
    <title>16Z077/087 Registers</title>
    <filename>group____Z077__REGISTERS.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_MODER</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_INT_SRC</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_INT_MASK</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_IPGT</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_IPGR1</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_IPGR2</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_PACKLEN</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_COLLCONF</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_TX_BDNUM</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_CTRLMODER</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_MIIMODER</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_MIICMD</name>
      <anchor>a11</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_MIIADR</name>
      <anchor>a12</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_MIITX_DATA</name>
      <anchor>a13</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_MIIRX_DATA</name>
      <anchor>a14</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_MIISTATUS</name>
      <anchor>a15</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_MAC_ADDR0</name>
      <anchor>a16</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_MAC_ADDR1</name>
      <anchor>a17</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_HASH_ADDR0</name>
      <anchor>a18</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_HASH_ADDR1</name>
      <anchor>a19</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_TXCTRL</name>
      <anchor>a20</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_Z087_NEWREGS</name>
    <title>16Z087 (MEN) specific Registers</title>
    <filename>group____Z087__NEWREGS.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_GLOBALRST</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_BDSTART</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_RXEMPTY0</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_RXEMPTY1</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_TXEMPTY0</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_TXEMPTY1</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_RXBDSTAT</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_SMBCTRL</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_COREREV</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_RXERRCNT1</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_RXERRCNT2</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_TXERRCNT1</name>
      <anchor>a11</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_REG_TXERRCNT2</name>
      <anchor>a12</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_MACRO1</name>
    <title>Macros acting on the Rx/Tx Descriptors</title>
    <filename>group____MACRO1.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z077_SET_TBD_ADDR</name>
      <anchor>a0</anchor>
      <arglist>(n, adr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_SET_RBD_ADDR</name>
      <anchor>a1</anchor>
      <arglist>(n, adr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_GET_RBD_ADDR</name>
      <anchor>a2</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_GET_TBD_ADDR</name>
      <anchor>a3</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_SET_TBD_FLAG</name>
      <anchor>a4</anchor>
      <arglist>(n, f)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_SET_RBD_FLAG</name>
      <anchor>a5</anchor>
      <arglist>(n, f)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_CLR_TBD_FLAG</name>
      <anchor>a6</anchor>
      <arglist>(n, f)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_CLR_RBD_FLAG</name>
      <anchor>a7</anchor>
      <arglist>(n, f)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_GET_RBD_FLAG</name>
      <anchor>a8</anchor>
      <arglist>(n, f)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_GET_TBD_FLAG</name>
      <anchor>a9</anchor>
      <arglist>(n, f)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_SET_TBD_LEN</name>
      <anchor>a10</anchor>
      <arglist>(n, l)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_GET_TBD_LEN</name>
      <anchor>a11</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_GET_RBD_LEN</name>
      <anchor>a12</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_CLR_RBD_LEN</name>
      <anchor>a13</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_EMPTY</name>
      <anchor>a14</anchor>
      <arglist>(n)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_Z077_REGBITS</name>
    <title>Register Bit definitions</title>
    <filename>group____Z077__REGBITS.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TOTAL_BD</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MAXBUF_LEN</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_ETHT1</name>
    <title>message levels for ethtool usage</title>
    <filename>group____ETHT1.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>ETHT_MESSAGE_OFF</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ETHT_MESSAGE_LVL1</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ETHT_MESSAGE_LVL2</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ETHT_MESSAGE_LVL3</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_TXDB</name>
    <title>Tx Buffer Descriptor Bits</title>
    <filename>group____TXDB.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_READY</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_IRQ</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_WRAP</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_PAD</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_CRC</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_UNDERRUN</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_RETRY</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_RETLIM</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_LATECOL</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_DEFER</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_TX_BD_CARRIER</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_RXDB</name>
    <title>Rx Buffer Descriptor Bits</title>
    <filename>group____RXDB.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_EMPTY</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_IRQ</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_WRAP</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_MISS</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_OVERRUN</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_INVSYMB</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_DRIBBLE</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_TOOLONG</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_SHORT</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_CRCERR</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_RX_BD_LATECOL</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_RXST</name>
    <title>Rx status Register bits</title>
    <filename>group____RXST.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>REG_RXSTAT_REV</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>REG_RXSTAT_RXEMPTY</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_COREREV</name>
    <title>Core Revision Bits</title>
    <filename>group____COREREV.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>REG_COREREV_REVISION</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>REG_COREREV_IRQNEWEN</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_MODER</name>
    <title>MODER Register bits descripton</title>
    <filename>group____MODER.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_RXEN</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_TXEN</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_NOPRE</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_BRO</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_IAM</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_PRO</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_IFG</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_LOOPBCK</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_NOBCKOF</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_EXDFREN</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_FULLD</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_RST</name>
      <anchor>a11</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_DLYCRCEN</name>
      <anchor>a12</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_CRCEN</name>
      <anchor>a13</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_HUGEN</name>
      <anchor>a14</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_PAD</name>
      <anchor>a15</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_RECSMALL</name>
      <anchor>a16</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_RX_IDLE</name>
      <anchor>a17</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_TX_IDLE</name>
      <anchor>a18</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_HD_AVAL</name>
      <anchor>a19</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_NIOSSHIFT</name>
      <anchor>a20</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MODER_COMPLI</name>
      <anchor>a21</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_INTS</name>
    <title>Interrupt Source Register bits descripton</title>
    <filename>group____INTS.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_TXB</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_TXE</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_RXF</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_RXE</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_BUSY</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_TXC</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_RXC</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_INTM</name>
    <title>Interrupt Mask Register bits descripton</title>
    <filename>group____INTM.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_MASK_TXB</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_MASK_TXE</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_MASK_RXF</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_MASK_RXE</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_MASK_BUSY</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_MASK_TXC</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_INT_MASK_RXC</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_CTRLMODER</name>
    <title>Control Module Mode Register</title>
    <filename>group____CTRLMODER.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_CTRLMODER_PASSALL</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_CTRLMODER_RXFLOW</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_CTRLMODER_TXFLOW</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_MODERMII</name>
    <title>MII Mode Register</title>
    <filename>group____MODERMII.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIIMODER_CLKDIV</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIIMODER_NOPRE</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIIMODER_RST</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_CMDR</name>
    <title>MII Command Register</title>
    <filename>group____CMDR.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIICMD_SCANSTAT</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIICMD_RSTAT</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIICMD_WCTRLDATA</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_MIIA</name>
    <title>MII Address Register</title>
    <filename>group____MIIA.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIIADDRESS_FIAD</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIIADDRESS_RGAD</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_MIISR</name>
    <title>MII Status Register</title>
    <filename>group____MIISR.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIISTATUS_LINKFAIL</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIISTATUS_BUSY</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>OETH_MIISTATUS_NVALID</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_MISCR</name>
    <title>Misc defines</title>
    <filename>group____MISCR.html</filename>
  </compound>
  <compound kind="group">
    <name>_BDDEF</name>
    <title>Buffer Descriptor defines</title>
    <filename>group____BDDEF.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z077_BDSIZE</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_NUM</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_NUM</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_BDALIGN</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_BDAREA</name>
    <title>address defines within one Buffer descriptor</title>
    <filename>group____BDAREA.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z077_BD_OFFS_STAT</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_BD_OFFS_LEN</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_BD_OFFS_ADR</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_TXBDBITS</name>
    <title>Transmit Buffer Descriptor bit definitions</title>
    <filename>group____TXBDBITS.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_RDY</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_IRQ</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_WRAP</name>
      <anchor>a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_PAD</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_CRC</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_UR</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_RETRY</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_RTRANS</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_LCOLL</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_DEFER</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_TBD_CSLOST</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_RXBDBITS</name>
    <title>Receive Buffer Descriptors bit definitions</title>
    <filename>group____RXBDBITS.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_EMP</name>
      <anchor>a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_IRQ</name>
      <anchor>a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_CFRM</name>
      <anchor>a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_MISS</name>
      <anchor>a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_OV</name>
      <anchor>a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_IS</name>
      <anchor>a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_DN</name>
      <anchor>a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_TL</name>
      <anchor>a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_SF</name>
      <anchor>a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_CERR</name>
      <anchor>a10</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_RBD_LC</name>
      <anchor>a11</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>_MACRO0</name>
    <title>Macros acting on Registers within the IP Core</title>
    <filename>group____MACRO0.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>Z077_DISABLE_IRQ</name>
      <anchor>a0</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_ENABLE_IRQ</name>
      <anchor>a1</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_CLR_MODE_FLAG</name>
      <anchor>a2</anchor>
      <arglist>(f)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>Z077_SET_MODE_FLAG</name>
      <anchor>a3</anchor>
      <arglist>(f)</arglist>
    </member>
  </compound>
</tagfile>
