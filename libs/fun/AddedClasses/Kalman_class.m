classdef Kalman_class
    
    properties
        value_out
        
        meas_prec
        meas_now
        
        std_meas_prec
        std_meas_now
        
        pred_prec
        pred_now
        
        std_pred_prec
        std_pred_now
        
        upd
        std_upd
        
        C
        A
        Q
        
    end
    
    methods
        function obj = Kalman_class(C,A,Q,state_init,std_state_init)
            obj.C=C;
            obj.A=A;
            obj.Q=Q;
            obj.pred_now=state_init;
            obj.std_pred_now=std_state_init;
            obj.pred_prec=obj.pred_now;
            obj.std_pred_prec=obj.std_pred_now;
            
        end
        
        function obj = AddNewMeas(obj,in1,std_in1)
            if ~isempty(obj.meas_now)
                obj.meas_prec=obj.meas_now;
                obj.std_meas_prec=obj.std_meas_now;
            end
            
            obj.meas_now=in1;
            obj.std_meas_now=std_in1;
        end        
        
        function obj=Predict(obj,dt)
            covar_now=diag(obj.std_pred_now.^2);
            
            [new_pred,newcovar]=Kalman_predict(obj.pred_now,covar_now,obj.A,obj.Q,dt);
            new_std=diag(newcovar).^(1/2);
            
            if ~isempty(obj.pred_now)
            obj.pred_prec=obj.pred_now;
            obj.std_pred_prec=obj.std_pred_now;
            end
            obj.pred_now=new_pred;
            obj.std_pred_now=new_std;
            
            obj.value_out=new_pred;
        end 
        
        function obj=Update(obj)
            covar_pred=diag(obj.std_pred_now.^2);
            covar_meas=diag(obj.std_meas_now.^2);
            [obj.upd,covar_upd]=Kalman_update(obj.pred_now,covar_pred,obj.C,obj.meas_now,covar_meas);
            obj.std_upd=diag(covar_upd).^(1/2);
            
            if ~isempty(obj.pred_now)
            obj.pred_prec=obj.pred_now;
            obj.std_pred_prec=obj.std_pred_now;
            end
            obj.pred_now=obj.upd;
            obj.std_pred_now=obj.std_upd;
            
            obj.value_out=obj.upd;
        end
        
        function obj=UpdateCAQ(obj,C,A,Q)
            obj.C=C;
            obj.A=A;
            obj.Q=Q;
        end
        
    end
end