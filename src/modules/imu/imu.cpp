#include "imu.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

//���Ľ�����һ�ֳ����������������̬���㷽����Mahony�Ļ����˲�����
//�˷�����Ч��ϣ���ܸ�ѧϰ����������������Ǵ���������
//������̬������˲�������֪ʶ���Ƽ�����Ԫ�������飬
//һ�ǡ����Ե�������Ŀǰ�ѳ����ڶ�����;���ǡ��������˲�����ϵ���ԭ����
//�����е����ۻ�������������Ѱ�ҡ�

//�ȶ���Kp��Ki���Լ�halfT ��
//Kp��Ki�����Ƽ��ٶȼ����������ǻ�����̬���ٶ�
//halfT ����̬����ʱ���һ�롣�˴�������̬�ٶ�Ϊ500HZ�����halfT Ϊ0.001
#define Kp 5.0f //1.6f //2.0f
#define Ki 0.0005f //0.001f //0.002f
#define halfT 0.001f
//��ʼ����Ԫ��
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
//������̬�������Ļ���
float exInt = 0, eyInt = 0, ezInt = 0;

//����Ϊ��̬���㺯����
//����gx��gy��gz�ֱ��Ӧ������Ľ��ٶȣ���λ�ǻ���/��;
//����ax��ay��az�ֱ��Ӧ������ļ��ٶ�ԭʼ����
//���ڼ��ٶȵ������ϴ󣬴˴�Ӧ�����˲��������
void imu_update(float gx, float gy, float gz, float ax, float ay, float az, struct vehicle_attitude_s *att)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    //�����ٶȵ�ԭʼ���ݣ���һ�����õ���λ���ٶ�
    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    //����Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء��������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء����������vx��vy��vz����ʵ���ǵ�ǰ�Ļ����������ϵ�ϣ����������������λ������(�ñ�ʾ������̬����Ԫ�����л���)
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    //����˵��һ�㣬���ٶȼ����������Ƚϴ󣬶����ڷ��й����У��ܻ�����Ӱ������������ԣ���ʱ���ڵĿɿ��Բ��ߡ�����������С���������ڻ�������ɢ�ģ���ʱ��Ļ��ֻ����Ư�Ƶ�����������Ҫ���ü��ٶȼ���õ���̬�����������ǻ�����̬��Ư�ơ�
    //�ڻ����������ϵ�ϣ����ٶȼƲ����������������ax��ay��az;���ݻ��ֺ����̬�������������������vx��vy��vz;����֮�������������������ݻ��ֺ����̬�ͼ��ٶȼƲ��������̬֮�����
    //���������������������(Ҳ����������)����ʾ��ex��ey��ez�����������������Ĳ���������������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����
    //�������ѧ�������ٶȰٿ�������ϸ���͡�
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    //����������л���
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
    //�ò���������PI����������ƫ��ͨ������Kp��Ki�������������Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶ�
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;
    //��Ԫ��΢�ַ��̣�ûɶ��˵���ˣ��������Ƽ�����ɣ��������۵Ķ������Ը���ĥ��ĥ
    //ʵ����ĥ�����ף��ǾͰ�ָ���Ĳ�����������������ٵõ���Ӧ����Ԫ�������ת����ŷ���Ǽ����ˡ��������黹�ǰ�����Ū���һ�㡣
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
    //��Ԫ����λ��
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    //eulerAngles[0] = (real32_T)atan2(Rot_matrix[7], Rot_matrix[8]);
    //eulerAngles[1] = -(real32_T)asin(Rot_matrix[6]);
    //eulerAngles[2] = (real32_T)atan2(Rot_matrix[3], Rot_matrix[0]);

    //att.roll = euler[0];
    //att.pitch = euler[1];
    //att.yaw = euler[2] + mag_decl;

    //angle->yaw += gyr->Z*Gyro_G*0.002f;
	//angle->rol = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 - AngleOffset_Pit; // pitch
	//angle->pit = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 - AngleOffset_Rol; // roll

    //roll
    //euler[0] = atan2 ( 2 * (q1 * q2 + 2 * q0 * q3), q0*q0 + q1*q1 - q2*q2 - q3*q3 )* 57.3; //roll
    att->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
    //pitch
    //euler[1] = asin(-2 * (q1 * q3 + 2 * q0* q2)) * 57.3; // pitch
    att->pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
    //yaw
    att->yaw = atan2f( 2 * (q0 * q1 + q2 * q3), q0*q0 - q1*q1 - q2*q2 + q3*q3 )*57.3;//yaw

    //�ɼ�����Ԫ������ŷ����
    // Q_ANGLE.Z = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* Rad; //yaw
    att->pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
    att->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3; //roll
    if(att->roll > 90 || att->roll < -90)
    {
        if(att->pitch > 0)
            att->pitch = 180 - att->pitch;
        if(att->pitch < 0)
            att->pitch =- (180 + att->pitch);
    }


}

#ifdef __cplusplus
}
#endif

