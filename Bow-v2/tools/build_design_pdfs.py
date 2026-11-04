"""Rebuild consolidated V1, V2 and V3.5 PDFs from their selected PCB documents."""
from pathlib import Path
from collections import defaultdict
from html import escape
import hashlib
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Table, TableStyle, Spacer, PageBreak, KeepTogether
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.graphics.shapes import Drawing, Line, String, Rect, Circle
from epro_model import model, key

ROOT=Path(__file__).resolve().parents[1]
pdfmetrics.registerFont(TTFont('BowCJK', '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'))
FONT='BowCJK'; NAVY=colors.HexColor('#16324F'); TEAL=colors.HexColor('#087E8B')
styles={
 'body':ParagraphStyle('body',fontName=FONT,fontSize=9,leading=14,spaceAfter=7,wordWrap='CJK'),
 'cell':ParagraphStyle('cell',fontName=FONT,fontSize=8,leading=10,wordWrap='CJK'),
 'small':ParagraphStyle('small',fontName=FONT,fontSize=8,leading=12,spaceAfter=6,wordWrap='CJK'),
 'title':ParagraphStyle('title',fontName=FONT,fontSize=19,leading=26,textColor=NAVY,spaceAfter=13),
 'section':ParagraphStyle('section',fontName=FONT,fontSize=12,leading=17,textColor=TEAL,spaceBefore=9,spaceAfter=8),
}
def para(t,style='body'):return Paragraph(escape(str(t)).replace('\n','<br/>'),styles[style])
def table(headers,rows,widths):
 t=Table([[para(x,'cell') for x in row] for row in [headers]+rows],colWidths=widths,repeatRows=1,hAlign='LEFT')
 t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#DCE9F0')),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#F3F7FA')]),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),3),('BOTTOMPADDING',(0,0),(-1,-1),3),('LINEBELOW',(0,0),(-1,0),.6,TEAL),('LINEBELOW',(0,-1),(-1,-1),.4,colors.HexColor('#CAD8E2'))]))
 return t

ROLES={
'R1':'补桥上臂','R2':'补桥下臂','R3':'全桥正输入串联滤波','R4':'全桥负输入串联滤波','R5':'半桥信号串联滤波','R6':'补桥中点串联滤波','R7':'ADC CS 上拉（V1 为已知误接）','R8':'IMU CS 上拉','R9':'UART DATA 上拉','R10':'UART 串联阻尼','R11':'NRST 上拉','R12':'LTC 限流 / 低通','R13':'LTC 信号偏置','R14':'偏置分压下臂','R15':'偏置分压上臂','R16':'阈值分压下臂','R17':'阈值分压上臂','R18':'比较器正反馈迟滞','R19':'LTC 数字输出串联','R20':'LED 限流','R21':'LTC 外部输入下拉；在 C20 之前',
'C1':'U4.1 输入去耦','C2':'U4.5 输出电容','C3':'紧靠 U3.4 / U3.5','C4':'MCU 储能；靠 U3.4 / U3.5','C5':'紧靠 U1.15 / AVDD','C6':'紧靠 U1.13 / DVDD','C8':'REFOUT 去耦；不接桥路','C9':'紧靠 U2.8 / VDD','C10':'紧靠 U2.5 / VDDIO','C12':'全桥差模滤波','C13':'半桥差模滤波','C14':'全桥正输入对地滤波','C15':'全桥负输入对地滤波','C16':'半桥信号对地滤波','C17':'补桥中点对地滤波','C18':'NRST 滤波；靠 U3.6','C20':'LTC 隔直；无极性，不是电气隔离','C21':'LTC 信号低通；靠 U5.4','C22':'偏置滤波；不接阈值节点','C23':'紧靠 U5.5 电源脚',
}
ANCHORS={'U1.1':'H_ADC_N','U1.2':'H_ADC_P','U1.3':'F_ADC_P','U1.4':'F_ADC_N','U1.8':'ADC_DRDY','U1.9':'ADC_CS','U1.10':'SPI_MISO','U1.11':'SPI_MOSI','U1.12':'SPI_SCK','U1.16':'ADC_REFOUT','U2.12':'IMU_CS','U2.4':'IMU_INT1','U3.2':'UART_TX','U3.6':'NRST','U3.7':'LTC_DIG','U3.18':'SWDIO','U3.19':'SWCLK','R1.1':'H_MID','R3.1':'F_PLUS','R4.1':'F_MINUS','R5.1':'H_RAW','R10.1':'DATA','C1.1':'5V_IN','C20.1':'LTC_IN','R12.1':'LTC_AC','U5.4':'LTC_SIG','R14.1':'LTC_BIAS','U5.3':'LTC_TH','U5.1':'LTC_OUT','LED1.1':'LED_A','LED1.2':'LED_STATUS_N'}

def prepare(v):
 m,pcb,sch=model(v,'PCB'+('3' if v=='v3.5' else v[1:]))
 names={};p=m['parts']
 for anchor,name in ANCHORS.items():
  r,n=anchor.split('.')
  if r in p and n in p[r]['pins']:
   net=p[r]['pins'][n]['net']
   if net and net not in ['GND','3V3']:names[net]=name
 m['alias']=names
 return m

def label(m,net):
 if not net:return 'NC / 独立悬空'
 alias=m['alias'].get(net,net)
 return alias if alias==net else alias+'\n('+net+')'

def pin_rows(m,ref):
 part=m['parts'][ref];out=[]
 for n,p in sorted(part['pins'].items(),key=lambda x:key(x[0])):
  net=p['net'];peers=sorted([x for x in m['nets'].get(net,[]) if x!=ref+'.'+n],key=key)
  if net in ['GND','3V3']:
   detail='公共地平面' if net=='GND' else '公共 3.3V 电源 / 桥激励'
   local={'U1.13':'C6.2','U1.15':'C5.2','U2.5':'C10.2','U2.8':'C9.2','U3.4':'C3.2'+('、C4.1' if 'C4' in m['parts'] else ''),'U3.5':'C3.1'+('、C4.2' if 'C4' in m['parts'] else ''),'U5.5':'C23.2'}
   if ref+'.'+n in local:detail+='；本地 '+local[ref+'.'+n]
  else:detail='、'.join(peers) if peers else '不与其他电气网络连接'
  if ref in ['J1','J3'] and int(n)>(3 if m['version']=='v1' and ref=='J1' else 4 if ref=='J1' else 5):detail='机械固定焊盘；不是信号端口'
  if m['version']=='v1' and ref+'.'+n in ['U1.9','U2.1']:detail+='；已知错误：IMU SDO 与 ADC CS 同网'
  out.append([n,p['name'],label(m,net),detail])
 return out

def ltc_drawing(v):
 d=Drawing(500,115)
 def line(x1,y1,x2,y2):d.add(Line(x1,y1,x2,y2,strokeColor=NAVY,strokeWidth=1))
 def text(x,y,t,size=9):d.add(String(x,y,t,fontName=FONT,fontSize=size,fillColor=NAVY))
 line(20,80,165,80);line(165,68,165,92);line(173,68,173,92);line(173,80,225,80)
 d.add(Rect(225,74,60,12,strokeColor=NAVY,fillColor=colors.white));line(285,80,450,80)
 text(20,100,'J1.1 / LTC_IN');text(145,100,'C20 1uF');text(225,100,'R12 4.7kΩ');text(350,100,'LTC_SIG → U5.4 / IN-')
 if v=='v3.5':
  line(90,80,90,62);d.add(Circle(90,80,2,fillColor=NAVY));d.add(Rect(84,30,12,32,strokeColor=TEAL,fillColor=colors.white));line(90,30,90,18);line(78,18,102,18);line(82,14,98,14);text(108,42,'R21 2.2kΩ');text(108,17,'GND')
 else:text(35,38,'此版本无输入下拉；不使用 LTC 时选择 no_ltc 固件。')
 return d

def build(v):
 m=prepare(v);p=m['parts'];story=[];titles=[]
 def page(title):
  if story:story.append(PageBreak())
  titles.append(title);story.append(para(title,'title'))
 def note(t):story.append(para(t))
 def pins(ref,title=None):
  story.append(para(title or (ref+' / '+p[ref]['model']),'section'))
  story.append(table(['脚号','名称','网络 / 工程名','直接连接 / 用途'],pin_rows(m,ref),[30,115,100,270]))
 def passive(prefix):
  return sorted([r for r in p if reffamily(r)==prefix],key=key)
 page('器件清单 · 芯片、接口与保护')
 note(f'依据 {v}.epro2 中的 {m["board"]} 及其对应原理图；一个工程中其他板卡不属于本版。以下端号和 BOM 按实际工程记录，同名网络相连。')
 if v=='v1':note('V1 是历史硬件记录：R7 错接 ADC_DRDY；U2.1 错接 ADC_CS。本文保留错误以匹配实物，不将其静默改成设计目标。J1/J3 不能沿用 V2/V3 线序。')
 if v=='v2':note('V2 已修正 R7 和 IMU MISO，包含 LTC 输入；无状态 LED、无 IMU INT1 连线，C4 未放置。')
 if v=='v3.5':note('V3.5 在 V3 上新增 R21=2.2kΩ：J1.1 / LTC_IN 对 GND，下拉位于 C20 输入侧。桥输入网络改名为 H1/F1/F2；其余现有器件端子拓扑一致。')
 rows=[]
 for r in sorted([r for r in p if not r.startswith(('R','C'))],key=key):
  z=p[r];rows.append([r,z['model'],z['footprint'],z['supplier']])
 story.append(table(['位号','型号','工程封装名称','商城编号'],rows,[38,165,245,67]))
 note('封装名称保留工程原文。U1 底部焊盘编号17=EP，必须接地。GH连接器超出信号脚数的焊盘为机械固定脚。')
 if 'U5' in p:note('U5 型号为 TLV7011DBVR，工程库封装名包含 TSOT-23-5；装配前按厂家 DBV 尺寸及脚号核对，不能只凭名称匹配。')
 page('器件清单 · 电阻与电容')
 groups=defaultdict(list)
 for r in passive('R')+passive('C'):
  z=p[r];groups[(z['value'],z['model'],z['footprint'],z['supplier'])].append(r)
 rows=[[', '.join(rs),a[0]+' / '+a[2],a[1],a[3]] for a,rs in groups.items()]
 story.append(table(['位号','数值 / 封装','制造商型号','商城编号'],rows,[135,90,220,70]))
 note('未出现的位号为空号或未放置器件，不应凭编号补装。BOM 是工程指定物料，实物换料需另行记录。应变片属于板外材料，不计入 PCB 贴装 BOM。')
 if v=='v1':note('V1：C1=2.2uF、C2=10uF，未放 C4；C12/C13 与 C14-C17 为原工程 X7R 料号，不等同后续版本的 C0G。')
 else:note('C1/C2 为10uF X5R，V3.5 的 C4同料号；100nF去耦为X7R，10nF和1nF模拟滤波采用C0G。0402电容应核对直流偏压后的有效容量。')
 page('引脚连接 · U1 ADC')
 pins('U1')
 note('半桥差分：AIN1 - AIN0，MUX_CFG=0x10；全桥：AIN2 - AIN3，MUX_CFG=0x23。桥激励与 ADC AVDD 共用3V3，数字换算以AVDD为参考。REFOUT仅接C8，不驱动应变桥。')
 note('AIN4/5未用，GPIO0/1关闭；AIN6未用，GPIO2关闭。U1.8配置为DRDY输出。V1的R7实际接在此网络，后续版本才移至CS。')
 page('引脚连接 · U2 IMU 与 U4 稳压器')
 pins('U2');pins('U4')
 note('U4.1与U4.3同接输入5V，上电使能；U4.4是NC，不能接5V或GND。U4无裸露焊盘。C1/C2分别靠输入/输出，地回路应短。')
 page('引脚连接 · U3 MCU')
 pins('U3')
 note('48MHz内部时钟；PC14用于USART1_TX，PC15用于ADC_CS。PA13/PA14保留SWD。V2使用FIFO轮询；V3.5沿用V3固件的PA6中断及PA4 LED。')
 if v=='v1':note('当前firmware目录没有V1专用构建；不能直接宣称v2固件兼容未经修复的V1实物，尤其IMU SDO与ADC_CS错误网络。')
 page('引脚连接 · 全部电阻')
 story.append(table(['器件 / 数值','1脚网络','2脚网络','用途'],[[r+' / '+p[r]['value'],label(m,p[r]['pins']['1']['net']),label(m,p[r]['pins']['2']['net']),ROLES.get(r,'')] for r in passive('R')],[86,123,123,183]))
 note('R1/R2保留1kΩ；匹配比值和相近温度比单纯降低阻值更重要。新增下拉并不替代比较器正反馈迟滞。')
 page('引脚连接 · 全部电容')
 story.append(table(['器件 / 数值','1脚网络','2脚网络','用途 / 位置'],[[r+' / '+p[r]['value'],label(m,p[r]['pins']['1']['net']),label(m,p[r]['pins']['2']['net']),ROLES.get(r,'')] for r in passive('C')],[86,123,123,183]))
 note('C3=100nF是MCU本地高频去耦；C4（仅本版存在时）为较大储能电容，不能代替C3。C5/C6分别靠ADC AVDD/DVDD。C8不改变当前使用AVDD参考的选择。')
 page('引脚连接 · 比较器、二极管与 LED')
 if 'U5' in p:pins('U5')
 for r in ['D1','D2','D3','LED1']:
  if r in p:pins(r)
 note('D1用于DATA的ESD保护，D2用于5V输入瞬态保护，不是稳压器。D3（如有）把LTC_SIG钳位到电源轨；掉电时持续输入可能反灌3V3。')
 if 'U5' in p:note('U5.3=IN+，接LTC_TH；U5.4=IN-，接LTC_SIG。R18=330kΩ提供正反馈；LTC_BIAS与LTC_TH是不同网络，不能短接。')
 page('外部接线 · USB-TTL、应变桥与 ST-LINK')
 for r in ['J1','J2','J3']:pins(r)
 note('电脑接收：DATA接USB-TTL的RX/RXD（3.3V逻辑），适配器TX悬空；板端5V接USB的5V供电，所有GND共地。UART=1500000 baud、8N1、无流控。')
 note('烧录：SWDIO→ST-LINK SWDIO，SWCLK→SWCLK，NRST→NRST，GND→GND。板上3V3只接调试器的VTref/VAPP参考输入；若调试器3.3V为电源输出，则不连接，避免电源并联。按上表端号，不凭插头左右方向。')
 note('应变桥：J2.1=E-，J2.2=E+，J2.3为半桥中点，J2.4/5为全桥两中点。全桥四栅接在E+/E-之间，两分压中点分别接J2.4/5；半桥两栅串接E+/E-，中点接J2.3。桥阻值和安装结构按实物，ADC不会自动完成力标定。')
 page('工作配置、局部示意与核对结果')
 if 'U5' in p:
  story.append(ltc_drawing(v))
  if v=='v3.5':note('R21与R9/R20复用FRC0402F2201TS（C2906865，0402）。在±1V输入下，R21支路峰值电流约0.455mA。它给未连接输入提供直流回路并降低拾噪阻抗；有效LTC幅度仍取决于信号源输出阻抗，需验证25fps持续锁定。')
  else:note('本版LTC输入没有R21下拉。实测悬空输入可能造成大量边沿中断并使IMU FIFO满；不用LTC时选no_ltc固件。有效信号由C20隔直后经R12、R13偏置进入比较器。')
 note('ADC模拟滤波：R3-R6=100Ω；差分电容C12/C13=10nF；对地C14-C17=1nF。忽略桥源阻抗时差模极点约75.8kHz，共模约1.59MHz；实际桥源阻抗会降低截止频率，不能把该网络称为500Hz抗混叠滤波。')
 if 'U5' in p:note('LTC滤波：R12=4.7kΩ、C21=1nF，简单估算极点约34kHz（实际受偏置及信号源阻抗影响）。R16/R17=10kΩ、R18=330kΩ，理想3.3V输出下外部迟滞约49mV，另有比较器自身迟滞。')
 note('当前V2/V3固件：ADC Mode3、OSR256、PGA32、AVDD参考、global chop关闭；TIM3每500us交替调度，目标每桥1000SPS。IMU低噪声HiRes FIFO，1600Hz，AAF开启、UI LPF约400Hz；按固定±32g/±4000°/s比例换算。SPI默认3MHz。')
 if v=='v3.5':note('V3.5与V3固件引脚兼容，可使用v3或v3_no_ltc；INT1为PA6，两帧FIFO水位，中断外保留5ms轮询兜底。PA4低有效LED正常每秒短亮60ms，故障快闪。新增R21不要求修改引脚配置。')
 if v=='v2':note('V2使用v2或v2_no_ltc；IMU无INT1连线，由主循环轮询FIFO。')
 note('四个现有固件均由host/read.py接收；--csv out/record01.csv统一保存IMU、ADC、LTC、STATUS及FIFO状态，不做电脑端平滑。每种数据保留独立时间戳，不强行对齐。')
 if v=='v1':note('V1重点：除R7误接外，U2.1与U1.9/U3.3同网；它没有连接到U1.10/U3.16的MISO网。这是工程级问题，实物若已飞线修改，请以实测为准。')
 note('核对范围：已逐组比较原理图导线端子与PCB网络分配，未发现两者之间的分组矛盾；这不等同于ERC/DRC、走线连续性、焊接或制造放行。本文未修改EDA、固件或制造导出。V3/V4旧版PDF中的固件“待实施”描述不代表当前firmware目录状态。')
 sha=hashlib.sha256((ROOT/'design'/f'{v}.epro2').read_bytes()).hexdigest()
 note(f'来源：design/{v}.epro2 / {m["board"]}；器件型号和引脚名来自该工程库。电气规格请对照specs中的厂家手册。工程SHA256：{sha[:32]}\n{sha[32:]}')
 out=ROOT/'design'/f'Bow-{v}_Design_and_Pin_Connections.pdf'
 def footer(c,doc):
  c.setFillColor(NAVY);c.setFont(FONT,10);c.drawString(40,813,f'BOW-{v} / 完整设计与引脚连接')
  c.setFont(FONT,8);c.setFillColor(colors.HexColor('#5A6A79'));c.drawString(40,798,f'2026-11-04  |  {v}.epro2 / {m["board"]}  |  工程同步版')
  c.setStrokeColor(TEAL);c.line(40,790,555,790);c.line(40,40,555,40)
  c.drawString(40,27,'BOM · 每个端子连接 · 外部接线 · 工作配置');c.drawRightString(555,27,str(doc.page))
 doc=SimpleDocTemplate(str(out),pagesize=(595.28,841.89),leftMargin=40,rightMargin=40,topMargin=63,bottomMargin=51,title=f'Bow-{v} 完整设计与引脚连接',author='Bow hardware project')
 doc.build(story,onFirstPage=footer,onLaterPages=footer)
 print(out)

def reffamily(r):return ''.join(x for x in r if not x.isdigit())
if __name__=='__main__':
 for version in ['v1','v2','v3.5']:build(version)
