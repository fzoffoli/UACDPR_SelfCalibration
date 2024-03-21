% Method to check for Cable-tension feasibility: a warning will be
% displayed if cable tension goes below a lower boundary (LowerBound).

% Can be called without Lower boundary specification to use value stored in
% UACDPR.LowerBound.

% If a new Lower Bound is specified the new value will be also written into
% UACDPR.Trasmission.LowerBound

function UACDPR = TauCheck(UACDPR,LowerBound)

    if nargin==1

        if any(UACDPR.Trasmission.CableTensions<=UACDPR.Trasmission.LowerBound)
            warning('Cable Tension below lower boundary')
        end

    elseif nargin==2

        UACDPR.Trasmission.LowerBound=LowerBound;
        if any(UACDPR.Trasmission.CableTensions<=UACDPR.Trasmission.LowerBound)
            warning('Cable Tension below lower boundary')
        end

    end

            

end

