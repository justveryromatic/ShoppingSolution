/* bug
 * 1.����������������㿪ͷʱ����
 * 
 * 
*/


using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using SpeakLib;                    //��������ģ��
using System.IO.Ports;             //���봮�ڵ���
using ProductInfomationLib;        //������Ʒ��Ϣ��
using Map;
using RouteLibrary;
using System.Runtime.InteropServices;
namespace GouTu
{
    public partial class Form1 : Form
    {

        #region variable
        ProductInfomationLib.Product[] products;
        SerialPort serial = new SerialPort();
        private int proid1 = -1;
        private int proid2 = -1;
        private int proid3 = -1;

        int nowPoint = 0;
        int lastNode = 0;
        int nextNode = 0;

        const int QueueLength = 10;
        byte[] Queue = new byte[QueueLength];
        int QueueHead = 0;//Point to the value at the head of the queue
        int QueueTail = 0;//After the last item  //Empty queue when QueueHead == QueueTail
        #endregion

        public Form1()
        {
            InitializeComponent();

            //��ʼ����Ʒ��Ϣ
            products = ProductInfomationLib.Product.GetProducts();

            //��ʼ����ͼ������Ϣ
            Map.Class1.CreateMap();
            RouteLibrary.Class1.SetPAndPosition();

            //��ʼ���˿���Ϣ
         //   InitPort();
        }

        
        #region �����Ʒ��Ϣ
        private void OutputProductInfo(double index, bool inway)
        {
            int sink = -1;
            if (inway)                          //�Ӵ�������
            {
                sink = (int)index;
                Paint(sink);

            }
            else                                //������������
            {
                for (int i = 0; i <= 29; i++)
                    if (products[i].labelnum == index)
                        sink = i;

                Paint(sink);
            }
        }

        private void Paint(int index)
        {
            //��Ʒ����
            this.tbxName.Text = products[index].name.ToString();

            //��Ʒ����
            if (products[index].type == 0)
                this.tbxType.Text = "ʳ��";
            else
                this.tbxType.Text = "ѧϰ������Ʒ";

            //��Ʒ�۸�
            this.tbxPrice.Text = Convert.ToString(products[index].price) + "Ԫ";

            //��Ʒ���ܺ�
            this.tbxShelf.Text = "��" + (((index + 1) % 6 == 0) ? ((index + 1) / 6) : ((index + 1) / 6 + 1)).ToString() + "����";

            //��Ʒ����
            if (((index) / 3 + 1) % 2 == 0)
                this.tbxStair.Text = "��2��";
            else
                this.tbxStair.Text = "��1��";

            EnsureOrder(index);

            RichPaint(index);
        }

        private void EnsureOrder(int index)
        {
            if (proid1 == -1)
                proid1 = index;
            else if (proid2 == -1)
                proid2 = index;
            else if (proid3 == -1)
                proid3 = index;

        }

        private void btnInfo_Click(object sender, EventArgs e)
        {
            SpeakLib.Class1.Read(
                                "��Ʒ��Ϣ���£�"
                                + "����" + this.tbxName.Text.ToString()
                                + "����" + this.tbxType.Text.ToString()
                                + "�۸�" + this.tbxPrice.Text.ToString()
                                + "���ܱ��" + this.tbxShelf.Text.ToString()
                                + "����" + this.tbxStair.Text.ToString()
                                );
        }

        private void RichPaint(int indext)
        {
            if (proid1 == -1) //zero
            {

            }
            else if (proid2 == -1)//one
            {
                this.rtxMessage.Text =
                    ("��ǰ�ѹ�����Ʒ����\n\t1��\n"
                    + "��Ʒ���ƣ�\n\t" + products[proid1].name + "\n"
                    + "���ۣ�\n\t" + products[proid1].price + "Ԫ\n"
                    + "�ܼۣ�\n\t" + products[proid1].price + "Ԫ\n"
                    );

            }
            else if (proid3 == -1)//two
            {
                this.rtxMessage.Text =
                    ("��ǰ�ѹ�����Ʒ����\t2��\n"
                    + "��Ʒ1��\n\t" + products[proid1].name + "\n"
                    + "���ۣ�\n\t" + products[proid1].price + "Ԫ\n"
                    + "��Ʒ2��\n\t" + products[proid2].name + "\n"
                    + "���ۣ�\n\t" + products[proid2].price + "Ԫ\n"
                    + "�ܼۣ�\t" + (products[proid1].price + products[proid2].price) + "Ԫ\n"
                    );
            }
            else //three
            {
                this.rtxMessage.Text =
                   ("��ǰ�ѹ�����Ʒ����\t3��\n"
                   + "��Ʒ1��\n\t" + products[proid1].name + "\n"
                   + "���ۣ�\n\t" + products[proid1].price + "Ԫ\n"
                   + "��Ʒ2��\n\t" + products[proid2].name + "\n"
                   + "���ۣ�\n\t" + products[proid2].price + "Ԫ\n"
                   + "��Ʒ3��\n\t" + products[proid3].name + "\n"
                   + "���ۣ�\n\t" + products[proid3].price + "Ԫ\n"
                   + "�ܼۣ�\t" + (products[proid1].price + products[proid2].price + products[proid3].price) + "Ԫ\n"
                   );
            }
        }
        #endregion


        #region ������Ӧ����

        //�Ӽ���������Ʒ���
        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            double index = -1;
            TextBox tb = (TextBox)sender;
            if (tb.Text.IndexOf('#') == 1 | tb.Text.IndexOf('#') == 2)                            //��0��ʼ
            {
                index = Convert.ToDouble(tb.Text.Substring(0, (tb.Text.IndexOf('#')))) - 1;             //���index��0��ʼ
                OutputProductInfo(index, true);
            }
        }

        //��������
        private void tbxLabelnum_TextChanged(object sender, EventArgs e)
        {
            TextBox tb = (TextBox)sender;
            if (tb.Text.Length < 13)
                return;

            else
            {
                string num = tb.Text.Substring(0, 13);
                MessageBox.Show(num);
                OutputProductInfo(Convert.ToDouble(num), false);

                //������Ʒ3����Ϣ
                if (products[proid3].type == 0)
                    serial.Write("*$");
                else
                    serial.Write("*@");
            }
        }
        private void btnEnd_Click(object sender, EventArgs e)
        {
            SpeakLib.Class1.Read(this.rtxMessage.Text);
        }
    
        private void Form1_Load(object sender, EventArgs e)
        {

        }
        private void rtxMessage_TextChanged(object sender, EventArgs e)
        {

        }
        private void groupBox5_Enter(object sender, EventArgs e)
        {

        }
        private void groupBox4_Enter(object sender, EventArgs e)
        {

        }

        private void btnFollow_Click(object sender, EventArgs e)
        {
            InitPort();
            DaoGou();
            serial.Close();             //ע�⴫����Ϣ�󴮿ڱ���رգ�������һ�δ���ʱ�޷�ִ��InitPort������
            tbxLabelnum.Focus();
        }

        private void btnSeparate_Click(object sender, EventArgs e)
        {
            InitPort();

            serial.Close();             
        }
        #endregion

        #region �������

        private void InitPort()
        {
            serial.PortName = "COM4";                                    //���ں�                        
            serial.BaudRate = 9600;                                     //������
            serial.Handshake = System.IO.Ports.Handshake.None;          //����λ
            serial.Parity = Parity.None;                                //��żУ��
            serial.DataBits = 8;                                        //����λ
            serial.StopBits = StopBits.One;                             //ֹͣλ
            serial.ReadTimeout = 200;
            serial.WriteTimeout = 50;
            serial.Open();
        }

        private int QueueHeadNext()
        {
            return (QueueHead + 1) % QueueLength;
        }
        private int QueueTailNext()
        {
            return (QueueTail + 1) % QueueLength;
        }
        private bool QueuePush(byte item)
        {
            if (QueueTailNext() == QueueHead)//Able to store QueueLength-1 items at most
            {
              //  printDebug("Queue Full. ");
                return (false);
            }
            else
            {
                Queue[QueueTail] = item;
                QueueTail = QueueTailNext();
                return (true);
            }
        }
        private byte QueuePop()
        {
            if (QueueHead == QueueTail)
            {
              //  printDebug("... ");
            }
            while (QueueHead == QueueTail)
            {
            }
            byte ret = Queue[QueueHead];
            QueueHead = QueueHeadNext();
            return (ret);
        }

        private void TransmitPoints(int productIndex)   //���������·���Ͼ����������,ע��˺���ֻ���ڵ���Dijkstra��ʹ��
        {
            string allPoints = "";

            for (int i = 0; i < 44; i++)
                if (RouteLibrary.Class1.p[productIndex, i] != -1)
                    if (RouteLibrary.Class1.p[productIndex, i].ToString().Length == 2)
                        allPoints = allPoints + RouteLibrary.Class1.p[productIndex, i].ToString() + " ";
                    else
                    {
                        allPoints = allPoints + "0" + RouteLibrary.Class1.p[productIndex, i].ToString() + " ";
                    }

            string send = "*" + allPoints + "#";
            MessageBox.Show(send);
            serial.Write(send);

        }

        private void DaoGou()
        {
            if (proid1 == -1)
            {
                SpeakLib.Class1.Read("��û������ƷӴ��");
                MessageBox.Show("��û������ƷӴ��");
            }
            else if (proid2 == -1)
            {
                SpeakLib.Class1.Read("��ȱһ����ƷӴ��");
                MessageBox.Show("��ȱһ����ƷӴ��");
            }
            else
            {
                //1.���͵���Ʒ1·���Ľڵ�
                int proPoint1 = Product2MapPoint(proid1);
                int proPoint2 = Product2MapPoint(proid2);
            //    RouteLibrary.Class1.CreateNode(8, 13);
             //   RouteLibrary.Class1.CreateNode(9, 14);
           //     RouteLibrary.Class1.CreateNode(10, 15);
                RouteLibrary.Class1.Dijkstra(0);
                TransmitPoints(proPoint1);                                              //




                //2.���ڵȴ�����

                bool waitingSerial = true;
                    string recv = "";

                serial.DiscardInBuffer();
                while (waitingSerial)
                {
                    serial.BytesToRead

                    recv = serial.Read();
                   // serial.DiscardInBuffer();               //������ڻ�������


                    //1.�����ϰ�����·,������·�������ͽڵ�
                    if (recv != "")
                    {
                        MessageBox.Show("recv=");
                        MessageBox.Show(recv);
                        if (recv.IndexOf('!') != -1)
                        {
                            serial.DiscardInBuffer(); 
                            GetPoint(recv);
                            RouteLibrary.Class1.CreateNode(lastNode, nextNode);
                            RouteLibrary.Class1.Dijkstra(nowPoint);
                            TransmitPoints(proPoint1);
                        }
                            //�����nowPoint,lastNode,nextNode

                        else if (recv.IndexOf('@') != -1)
                        {
                            serial.DiscardInBuffer(); 

                            System.Threading.Thread.Sleep(2000); //������Ʒ1,����ͣ2s��������ֵ���д�ȷ��������Ϊȡ��Ʒ��ʱ�䣬Ȼ���͸���Ʒ��Ʒ���

                            if (products[proid1].type == 0)
                            {
                                serial.Write("*$");
                                MessageBox.Show("Send *$");
                            } //0��ʾʳ��

                            else
                            {
                                serial.Write("*@");
                                MessageBox.Show("Send *@");
                            } //0��ʾʳ��

                            RouteLibrary.Class1.Dijkstra(proPoint1); //����Ʒ1Ϊ��㣬���㵽��Ʒ2��·��
                            TransmitPoints(proPoint2);
                        }
                        else if (recv.IndexOf('#') != -1)
                        {
                                serial.DiscardInBuffer(); 
                            //�����nowPoint,lastNode,nextNode
                                GetPoint(recv);
                                RouteLibrary.Class1.CreateNode(lastNode, nextNode);
                                RouteLibrary.Class1.Dijkstra(nowPoint);
                                TransmitPoints(proPoint2);
                        }
                        else if (recv.IndexOf('$') != -1)
                        {
                            serial.DiscardInBuffer(); 

                            System.Threading.Thread.Sleep(2000);//������Ʒ2,����ͣ2s��������ֵ���д�ȷ��������Ϊȡ��Ʒ��ʱ�䣬Ȼ���͸���Ʒ��Ʒ���

                                if (products[proid2].type == 0)//0��ʾʳ��
                                    serial.Write("*$");
                                else
                                    serial.Write("*@");
                                waitingSerial = false;//����ѭ�����������׶�
                        }
                        

                    }

                    //ÿ��ִ�����Ҫ���recv
                    recv = "";
                }
            }
        }

        //�������յ����ϰ������Ϣʱ���������ǰ�㣬�ϰ�����������
        private void GetPoint(string recv)
        {
            char[] delimiterChars = { ' ' };

            string[] points = recv.Split(delimiterChars);

            for (int i = 0; i < points.Length; i++)
                MessageBox.Show(points[i]);
            if (points.Length < 3)
            {
                MessageBox.Show("������������");
                return;
            }

            nowPoint = Convert.ToInt32(points[1]);
            lastNode = Convert.ToInt32(points[2]);
            nextNode = Convert.ToInt32(points[3]);
        }

        //����Ʒ��Ŷ�Ӧ�������ϵĵ�
        private int Product2MapPoint(int proid)
        {
            int mapPoint = 0;
            switch (proid + 1)
            {
                case 1:
                case 4:
                    mapPoint = 8;
                    break;
                case 2:
                case 5:
                    mapPoint = 13;
                    break;
                case 3:
                case 6:
                    mapPoint = 18;
                    break;
                case 7:
                case 10:
                    mapPoint = 9;
                    break;
                case 8:
                case 11:
                    mapPoint = 14;
                    break;
                case 9:
                case 12:
                    mapPoint = 19;
                    break;
                case 13:
                case 16:
                    mapPoint = 10;
                    break;
                case 14:
                case 17:
                    mapPoint = 15;
                    break;
                case 15:
                case 18:
                    mapPoint = 20;
                    break;
                case 19:
                case 22:
                    mapPoint = 11;
                    break;
                case 20:
                case 23:
                    mapPoint = 16;
                    break;
                case 21:
                case 24:
                    mapPoint = 21;
                    break;
                case 25:
                case 28:
                    mapPoint = 12;
                    break;
                case 26:
                case 29:
                    mapPoint = 17;
                    break;
                case 27:
                case 30:
                    mapPoint = 22;
                    break;
                default:
                    break;
            }
            return mapPoint;
        }
        #endregion
    }
}


