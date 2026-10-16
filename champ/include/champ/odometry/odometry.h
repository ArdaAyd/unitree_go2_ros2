/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <quadruped_base/quadruped_base.h>
#include <macros/macros.h>

#include <geometry/geometry.h>

namespace champ
{
    class Odometry
    {
        QuadrupedBase *base_;
        geometry::Transformation prev_foot_position_[4];
        float prev_level_x_[4];
        float prev_level_y_[4];
        bool prev_foot_contacts_[4];
        float prev_theta_[4];
        unsigned long int prev_time_;
        champ::Velocities prev_vel_;
        float beta_;

        public:
            typedef unsigned long int Time;
            static inline Time now() { return time_us(); }

            Odometry(QuadrupedBase &quadruped_base, Time time = now()):
                base_(&quadruped_base),
                prev_foot_contacts_{1,1,1,1},
                prev_theta_{0,0,0,0},
                prev_time_(time),
                beta_(0.1)
            {
                for(unsigned int i = 0; i < 4; i++)
                {
                    prev_foot_position_[i] = base_->legs[i]->foot_from_base();
                    prev_level_x_[i] = prev_foot_position_[i].X();
                    prev_level_y_[i] = prev_foot_position_[i].Y();
                }
            }        
            
            bool allFeetInContact()
            {
                if(base_->legs[0]->in_contact() &&
                   base_->legs[1]->in_contact() &&
                   base_->legs[2]->in_contact() &&
                   base_->legs[3]->in_contact())
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            bool noFootInContact()
            {
                if(!base_->legs[0]->in_contact() &&
                   !base_->legs[1]->in_contact() &&
                   !base_->legs[2]->in_contact() &&
                   !base_->legs[3]->in_contact())
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            /**
             * @param roll, pitch govde yonelimi (IMU'dan). foot_from_base() ayagi
             * GOVDE cercevesinde veriyor; govde yururken saliniyor (olcumde roll
             * tepe-tepe 0.24 rad, pitch 0.14 rad) ve ~0.3 m bacak boyuyla bu,
             * ayagi govde cercevesinde +-4 cm oynatiyor. Bir adimin yolu ~15 cm
             * oldugundan telafi edilmezse bu donus, govde cevirisi sanilarak
             * odometriye karisiyor. Burada ayak vektoru once yatay (yer cekimine
             * hizali) cerceveye dondurulup oyle farki aliniyor.
             */
            void getVelocities(champ::Velocities &vel, float roll, float pitch,
                               Time now = champ::Odometry::now())
            {      
                // Dort ayak da yerdeyken govde hareketsiz DEGIL: tiris yuruyusunde
                // iki capraz ciftin degisimi sirasinda dort ayak birlikte temas ediyor
                // ve govde bu sirada ilerlemeye devam ediyor. Olcumde bu durum
                // orneklerin %36'sini olusturuyordu ve hepsinde hiz sifir sayildigi
                // icin odometri yolu sistematik olarak %33 eksik tahmin ediyordu.
                // Ayaklar yerdeyken govde yer degistirmesi ayak kinematiginden
                // hesaplanabilir, dolayisiyla bu durumun disarida birakilmasi icin
                // sebep yok. Yalnizca hicbir ayak yerde degilken (robot havada)
                // bilgi yok, orada sifir dogru.
                if(noFootInContact())
                {
                    vel.linear.x = 0.0;
                    vel.linear.y = 0.0;
                    vel.angular.z = 0.0;

                    prev_vel_.linear.x = 0.0;
                    prev_vel_.linear.y = 0.0;
                    prev_vel_.angular.z = 0.0;

                    return;
                }

                unsigned int total_contact = 0;
                float x_sum = 0;
                float y_sum = 0;
                float theta_sum = 0;

                // Ry(pitch) * Rx(roll): govde cercevesindeki vektoru yatay cerceveye tasir
                const float cr = cosf(roll),  sr = sinf(roll);
                const float cp = cosf(pitch), sp = sinf(pitch);

                for(unsigned int i = 0; i < 4; i++)
                {
                    geometry::Transformation current_foot_position = base_->legs[i]->foot_from_base();
                    
                    bool foot_in_contact = base_->legs[i]->in_contact();

                    const float bx = current_foot_position.X();
                    const float by = current_foot_position.Y();
                    const float bz = current_foot_position.Z();
                    const float lx = cp*bx + sp*sr*by + sp*cr*bz;
                    const float ly =              cr*by -    sr*bz;

                    float delta_x = (prev_level_x_[i] - lx);
                    float delta_y = (prev_level_y_[i] - ly);
                    
                    float current_theta = atan2f(lx, ly);
                    float delta_theta = (current_theta - prev_theta_[i]);

                    // Yalnizca ayak BIR ONCEKI dongude de temasta ise say. Dokunus
                    // anindaki ilk fark, ayagin salinim sirasinda one dogru yaptigi
                    // hareketi de iceriyor; bu, govdenin geriye gittigi gibi okunup
                    // yolu eksik tahmin ettiriyor. prev_foot_contacts_ bu amacla
                    // zaten saklaniyordu ama kullanilmiyordu.
                    if(foot_in_contact && prev_foot_contacts_[i])
                    {
                        total_contact += 1;
                        theta_sum += delta_theta;
                        x_sum += delta_x;
                        y_sum += delta_y;
                    }
                        
                    prev_foot_position_[i] = current_foot_position;
                    prev_level_x_[i] = lx;
                    prev_level_y_[i] = ly;
                    prev_foot_contacts_[i] = foot_in_contact;
                    prev_theta_[i] = current_theta;
                }

                double dt = (now - prev_time_) / 1000000.0;
                // zero division check
                if (dt == 0)
                    dt = 0.02;

                // Temas eden ayak sayisina bol. Onceki surumde bolen sabit 2 idi
                // ("her an tam iki ayak yerde" varsayimi); total_contact hesaplaniyor
                // ama kullanilmiyordu. Uc ayak temas ettiginde bu, yer degistirmeyi
                // 1.5 kat fazla sayiyordu.
                const float contact_divisor = (total_contact > 0) ? float(total_contact) : 1.0f;
                x_sum /= contact_divisor;
                y_sum /= contact_divisor;
                theta_sum /= contact_divisor;
                vel.linear.x =  ((1 - beta_) * ((x_sum * base_->gait_config.odom_scaler) / dt)) + (beta_ * prev_vel_.linear.x);
                vel.linear.y =  ((1 - beta_) * ((y_sum * base_->gait_config.odom_scaler) / dt)) + (beta_ * prev_vel_.linear.y);
                vel.angular.z = ((1- beta_ ) * (theta_sum / dt)) + (beta_ * prev_vel_.angular.z);
                
                prev_vel_ = vel;
                prev_time_ = now;
            }
    };
}

#endif

